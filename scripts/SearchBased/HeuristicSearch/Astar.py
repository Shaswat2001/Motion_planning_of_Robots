#!/usr/bin/env python3

import rospy
import yaml
import math
import numpy as np
import heapq
from yaml.loader import SafeLoader
from geometry_msgs.msg import Point
from motion_planning.msg import motion_planning
from utils.make_graph import Visualize
from utils.graph import Graph
from utils.Nodes import check_nodes,check_NodeIn_list,getSameNode,Node
from utils.heuristic import manhattan_heuristic,euclidean_heuristic
from utils.data_structure import PriorityQueue
from utils.non_holonomic import NonHolonomicDrive

class Astar:
    '''
    Implements the A* algorithm for a 2D environment
    '''
    def __init__(self,start,goal,graph,RPM,orientation_res,grid_res):

        self.start = start
        self.goal = goal
        self.graph = graph
        self.grid_res = grid_res
        self.orienatation_res = orientation_res

        self.drive = NonHolonomicDrive(RPM,graph.grid_size)

        self.orientation = {(360//orientation_res)*i:i for i in range(orientation_res)}

        self.f = np.full(shape=[grid_res[0],grid_res[1],orientation_res],fill_value=math.inf)
        self.cost = np.full(shape=[grid_res[0],grid_res[1],orientation_res],fill_value=math.inf)
        self.V = np.zeros(shape=[grid_res[0],grid_res[1],orientation_res])

        rospy.loginfo(f"The resolution of orientation is : {self.orienatation_res}")
        self.OPEN = PriorityQueue()
        self.CLOSED = []
        self.backtrack_node={} # used to extract the final path

    def plan(self):
        '''
        Finds the optimal path

        Returns:
        path -- the shortest path between start and goal node
        CLOSED -- List of nodes visited by the Algorithm
        '''
        
        start_node,start_orientation = self.start

        self.f[int(2*start_node.x)][int(2*start_node.y)][self.orientation[int(start_orientation)]] = 0
        self.cost[int(2*start_node.x)][int(2*start_node.y)][self.orientation[int(start_orientation)]] = 0

        self.OPEN.insert_pq(0, self.start)

        while self.OPEN.len_pq()>0:

            _,current_vtx = self.OPEN.pop_pq()

            self.CLOSED.append(current_vtx[0]) #Adding node to the CLOSED list

            if euclidean_heuristic(current_vtx[0],self.goal) <= 1.5: # Check if goal node is reached
                print("The goal node is found")
                return self.extract_path(current_vtx[0]),current_vtx[0]

            neighbour=self.drive.get_neighbours(current_vtx[0],current_vtx[1])

            for nbr_node,node_values in neighbour.items():
                
                # If the neighbour is not already visited and if not in Obstacle space
                if not self.graph.checkObstacleSpace(nbr_node) and not check_NodeIn_list(nbr_node,self.CLOSED):
                    movement_cost, new_orientation = node_values
                    new_orientation = new_orientation%360
                    if self.V[int(2*nbr_node.x)][int(2*nbr_node.y)][self.orientation[new_orientation]] == 0:

                        self.V[int(2*nbr_node.x)][int(2*nbr_node.y)][self.orientation[new_orientation]] = 1

                        self.cost[int(2*nbr_node.x)][int(2*nbr_node.y)][self.orientation[new_orientation]] = \
                            movement_cost + self.cost[int(2*current_vtx[0].x)][int(2*current_vtx[0].y)][self.orientation[current_vtx[1]]]

                        heurisitic_cost = manhattan_heuristic(self.goal,nbr_node)

                        self.f[int(2*nbr_node.x)][int(2*nbr_node.y)][self.orientation[new_orientation]] = \
                            self.cost[int(2*nbr_node.x)][int(2*nbr_node.y)][self.orientation[new_orientation]] + heurisitic_cost
                        
                        self.OPEN.insert_pq(self.f[int(2*nbr_node.x)][int(2*nbr_node.y)][self.orientation[new_orientation]],(nbr_node,new_orientation))
                        self.backtrack_node[nbr_node]={}
                        self.backtrack_node[nbr_node][self.f[int(2*nbr_node.x)][int(2*nbr_node.y)][self.orientation[new_orientation]]] = current_vtx[0]

                        if euclidean_heuristic(nbr_node,self.goal) <= 1.5: # Check if goal node is reached
                            print("The goal node is found")
                            return self.extract_path(nbr_node),nbr_node
                    else:

                        if self.f[int(2*nbr_node.x)][int(2*nbr_node.y)][self.orientation[new_orientation]] > self.f[int(2*current_vtx[0].x)][int(2*current_vtx[0].y)][self.orientation[current_vtx[1]]]:
                            self.f[int(2*nbr_node.x)][int(2*nbr_node.y)][self.orientation[new_orientation]] = self.f[int(2*current_vtx[0].x)][int(2*current_vtx[0].y)][self.orientation[current_vtx[1]]]
                            same_nbr_node = getSameNode(nbr_node,list(self.backtrack_node.keys()))
                            self.backtrack_node[same_nbr_node][self.f[int(2*nbr_node.x)][int(2*nbr_node.y)][self.orientation[new_orientation]]] = current_vtx[0]


        # If a path Doesn't exit
        print("The Goal coudnt be reached")
        return None,None
    
    def extract_path(self,vertex):
        '''
        Creates shortest path from start and goal node

        Returns:
        bkt_list -- List of nodes in the shortest path
        '''

        bkt_list=[]
        bkt_list.append(self.goal)
        node = vertex
        # loops till goal is not equal to zero
        while node!=0:
            for nbr,values in reversed(list(self.backtrack_node.items())):
                # if nbr and goal are same
                for cost,parent in values.items():

                    if check_nodes(nbr,node):
                        if not check_NodeIn_list(parent,bkt_list):
                            bkt_list.append(parent)

                        node=parent

                        if check_nodes(parent,self.start[0]):
                            node=0
                            return bkt_list

def get_grid_size(obstacle_list):

    grid_size = [[0,0]]*2
    for nodes in obstacle_list["boundary"]:

        x_min,x_max,y_min,y_max = nodes
        grid_size[0][0] = min(x_min,grid_size[0][0])  
        grid_size[0][1] = max(x_max,grid_size[0][1])  
        grid_size[1][0] = min(y_min,grid_size[1][0])  
        grid_size[1][1] = max(y_max,grid_size[1][1])  

    return grid_size


if __name__ == "__main__":

    rospy.init_node("Astar")

    mp_pub = rospy.Publisher("/motion_planning_results",motion_planning,queue_size=10)

    # Getting all the parameters from the server
    obstacles_file = rospy.get_param("obstacles_file")
    start_node=rospy.get_param("robot_pose")
    goal_pose = rospy.get_param("goal_position")
    RPM_val = rospy.get_param("RRM")
    orientation_pr = int(rospy.get_param("orientation_pr"))
    grid_pr = float(rospy.get_param("grid_pr"))

    with open(obstacles_file) as f:
        obs_locations = yaml.load(f, Loader=SafeLoader) 

    grid_size = get_grid_size(obs_locations)
    rospy.loginfo(f"The grid_x_min : {grid_size[0][0]} and grid_x_max : {grid_size[0][1]}")
    graph = Graph(grid_size,obs_locations,20,35)

    orientation_res = 360//orientation_pr
    grid_res = [int(abs(grid_size[0][0] - grid_size[0][1])/grid_pr+1),int(abs(grid_size[1][0] - grid_size[1][1])/grid_pr+1)]

    
    start_node = [float(x) for x in start_node[1:-1].split(" ")]
    start=(Node(start_node[0]*100,start_node[1]*100),start_node[-1])
    goal_node = [float(x) for x in goal_pose[1:-1].split(" ")]
    goal=Node(*(x for x in goal_node))
    

    if graph.checkObstacleSpace(start[0]) or graph.checkObstacleSpace(goal):
        rospy.logerr("The start or goal nodes are beyond the grid dimensions")

    RPM = [float(x) for x in RPM_val[1:-1].split(" ")]

    algorithm = Astar(start,goal,graph,RPM,orientation_res,grid_res)
    shortest_path,closest_vertex = algorithm.plan() 

    mp_data = motion_planning()
    mp_data.closest_vertex.x = closest_vertex.x
    mp_data.closest_vertex.y = closest_vertex.y
    mp_data.closest_vertex.z = 0
    
    for i in shortest_path:

        path_pt = Point()
        path_pt.x = i.x
        path_pt.y = i.y
        path_pt.z = 0
        mp_data.path.append(path_pt)

    mp_pub.publish(mp_data)

    plot_result = Visualize(start[0],goal,obs_locations,RPM,grid_size)
    plot_result.animate("Astar",None,shortest_path)

    for i in shortest_path:
        rospy.loginfo(f"The x : {i.x} and y : {i.y}")


