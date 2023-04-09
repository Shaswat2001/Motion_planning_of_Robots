#!/usr/bin/env python3

import rospy
import yaml
import math
import heapq
from yaml.loader import SafeLoader
from utils.graph import Graph
from utils.Nodes import check_nodes,check_NodeIn_list,Node
from utils.heuristic import manhattan_heuristic,euclidean_heuristic
from utils.data_structure import PriorityQueue

class Astar:
    '''
    Implements the A* algorithm for a 2D environment
    '''
    def __init__(self,start,goal,graph):

        self.start = graph.same_node_graph(start)
        self.goal = graph.same_node_graph(goal)
        self.graph = graph

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

        # vertices in the graph
        vertices=self.graph.get_vertices()

        past_cost={nodes:math.inf for nodes in vertices}
        past_cost[self.start]=0

        # Start Node with past cost is inserted into the queue
        self.OPEN.insert_pq(0, self.start)

        while self.OPEN.len_pq()>0:

            current_cost,current_vtx = self.OPEN.pop_pq()

            self.CLOSED.append(current_vtx) #Adding node to the CLOSED list

            if check_nodes(current_vtx,self.goal): # Check if goal node is reached
                print("The goal node is found")
                return self.extract_path(),self.CLOSED

            neighbour=self.get_neighbours(current_vtx)

            for nbr_node in neighbour:

                # If the neighbour is not already visited and if not in Obstacle space
                if not check_NodeIn_list(nbr_node,self.CLOSED):

                    # the tentatative_distance is calculated
                    tentatative_distance=past_cost[current_vtx]+euclidean_heuristic(current_vtx,nbr_node)
                    # If the past_cost is greater then the tentatative_distance
                    if past_cost[nbr_node]>tentatative_distance:
                        # the neigbour node along with its parent is added to the Dict
                        self.backtrack_node[nbr_node]=current_vtx
                        past_cost[nbr_node]=tentatative_distance
                        # Chosen heuristic is added to the tentatative_distance before adding it to the queue
                        tentatative_distance+=manhattan_heuristic(self.goal,nbr_node)
                        # Node along with the cost is added to the queue
                        self.OPEN.insert_pq(tentatative_distance, nbr_node)

                        #if the goal node is reached
                        if check_nodes(nbr_node,self.goal):
                            print("The goal node is found")
                            return self.extract_path(),self.CLOSED

        # If a path Doesn't exit
        print("The Goal coudnt be reached")
        return None,self.CLOSED
    
    def get_neighbours(self,current_node):

        nbr_list = []
        neighbour=current_node.get_neighbours()

        for nbr in neighbour:
            # If the neighbour is not in Obstacle space
            if not self.graph.checkObstacleSpace(nbr):

                nbr_list.append(self.graph.same_node_graph(nbr))

        return nbr_list
    
    def extract_path(self):
        '''
        Creates shortest path from start and goal node

        Returns:
        bkt_list -- List of nodes in the shortest path
        '''

        bkt_list=[]
        bkt_list.append(self.goal)
        node = self.goal
        # loops till goal is not equal to zero
        while node!=0:
            for nbr,parent in reversed(list(self.backtrack_node.items())):
                # if nbr and goal are same
                if check_nodes(nbr,node):

                    if not check_NodeIn_list(parent,bkt_list):
                        bkt_list.append(parent)

                    node=parent

                    if check_nodes(parent,self.start):
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

    obstacles_file = rospy.get_param("obstacles_file")
    with open(obstacles_file) as f:
        obs_locations = yaml.load(f, Loader=SafeLoader) 

    grid_size = get_grid_size(obs_locations)
    rospy.loginfo(f"The grid_x_min {grid_size[0][0]} and grid_x_max {grid_size[0][1]}")
    graph = Graph(grid_size,obs_locations,1,1)

    algorithm = Astar(Node(-430,-300),Node(-440,-300),graph)
    shortest_path,closed = algorithm.main()    

