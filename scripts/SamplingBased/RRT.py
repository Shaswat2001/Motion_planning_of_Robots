#!/usr/bin/env python3

import rospy
import yaml
import math
import random
import numpy as np
import heapq
from yaml.loader import SafeLoader
from geometry_msgs.msg import Point,Twist
from motion_planning.msg import motion_planning
from utils.make_graph import Visualize
from utils.graph import Graph
from utils.Nodes import Node
from utils.heuristic import manhattan_heuristic,euclidean_heuristic
from utils.non_holonomic import NonHolonomicDrive

class RRT:

    def __init__(self,start,goal,graph,RPM,orientation_res,tree_size = 10000,goalDist = 10):

        self.start =start
        self.goal = goal
        self.graph = graph
        self.goal_sample_rate = 0.05
        self.tree_size = tree_size
        self.goalDist = goalDist

        self.orienatation_res = orientation_res

        self.drive = NonHolonomicDrive(RPM,graph.grid_size)

        self.orientation = {(360//orientation_res)*i:i for i in range(orientation_res)}

        self.visited=[self.start]
        
    def plan(self):
        '''
        Performs the RRT algorithm

        Arguments:
        graph-- Object of class Graph
        start-- starting node (Object of class Node)
        goal-- goal node (Object of class Node)
        tree_size-- max_number of edges in the tree
        nodeDist-- distance between parent and new node
        maze_canvas-- array representing the entire grid

        returns:
        visited-- list of visited nodes
        tree-- list of edges
        '''

        # loops till size of tree is less than max_size
        for _ in range(self.tree_size):
            
            #  Randomly samples a node from the vertices in the graph
            sample_x=self.Sample()
            # nearest node to sample_x
            near_x=self.Nearest(self.visited,sample_x)
            # new node in the tree
            new_x,parent=self.Steer(sample_x,near_x)

            # if path between new node and nearest node is collision free
            if not self.checkNonHolonomicPath(parent["intermediate_nodes"]):

                new_x[0].parent = parent
                # add new node to visited list
                self.visited.append(new_x)

                # checks if node is in goal radius
                if self.check_Node_goalRadius(new_x[0]):
                    print("Goal Reached")
                    return self.extract_path(new_x),self.visited
        
        print("Goal Coudn't be reached")
        return None,None

    def Steer(self,x_sampNode,x_nearNode):
        '''
        Generates new Node in the grid

        Arguments:
        x_sampNode-- Node sampled from the grid
        x_nearNode-- Node nearest to x_sampNode

        Returns:
        x_new-- Object of class Node
        '''

        neighbour=self.drive.get_neighbours(x_nearNode[0],x_nearNode[1])
        min_cost = math.inf
        new_node = None
        parent = None

        for nbr_node,node_values in neighbour.items():

            new_orientation = node_values["orientation"]%360
            distance = euclidean_heuristic(nbr_node,x_sampNode)
            if distance < min_cost:
                min_cost = distance
                new_node = (nbr_node,new_orientation)
                parent = {"vertex":x_nearNode[0],
                        "orientation":x_nearNode[1],
                        "twist":node_values["twist"],
                        "intermediate_nodes":node_values["intermediate_nodes"]}

        return new_node,parent
    
    def Sample(self):

        if np.random.random() > self.goal_sample_rate:
                return self.graph.generate_random_node()

        return self.goal
    
    def check_Node_goalRadius(self,new_node):
        '''
        Checks if a Node is in the Goal radius
        '''
        if euclidean_heuristic(self.goal,new_node) < self.goalDist:
            return True
        else:
            return False

    def checkNonHolonomicPath(self,points):

        for (x,y) in points:

            if self.graph.checkObstacleSpace(Node(x,y)):
                return True

        return False

    @staticmethod
    def Nearest(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd[0].x - n.x, nd[0].y - n.y)
                                        for nd in node_list]))]
    
    def extract_path(self,node_end):
        
        goal_node = {"vertex":node_end[0],
                     "orientation":node_end[1],
                     "twist":None,
                     "intermediate_nodes":None}
        
        bkt_list=[goal_node]
        node = goal_node

        while node["vertex"].parent != None:

            node = node["vertex"].parent
            bkt_list.append(node)

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

    rospy.init_node("RRT")

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

    algorithm = RRT(start,goal,graph,RPM,orientation_res)
    shortest_path,visited_nodes = algorithm.plan() 

    mp_data = motion_planning()
    
    for i in shortest_path[1:]:

        path_pt = Point()
        path_pt.x = i["vertex"].x
        path_pt.y = i["vertex"].y
        path_pt.z = 0
        mp_data.path.append(path_pt)

        mp_data.orientation.append(i["orientation"])

        twist_pt = Twist()
        twist_pt.angular.z = i["twist"][0]
        twist_pt.linear.x = i["twist"][1]

        mp_data.turtlebot_twist.append(twist_pt)

    mp_pub.publish(mp_data)


    plot_result = Visualize(start[0],goal,obs_locations,RPM,grid_size)
    plot_result.animate("RRT",shortest_path)

    for i in shortest_path:
        rospy.loginfo(f"The x coordinate : {i['vertex'].x} and y coordinate : {i['vertex'].y} and orientation : {i['orientation']}")
