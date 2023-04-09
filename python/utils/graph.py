from utils.Nodes import Node,calculate_distance,check_nodes,check_NodeIn_list
from numpy import ones,vstack
from numpy.linalg import lstsq
import random
import numpy as np
import math

class Graph:
    '''
    This class decribes the entire map as a graph
    '''
    def __init__(self,grid_size,obs_nodes,clearance,radius):

        self.grid_size = grid_size
        self.graph = self.generate_graph()
        self.obs_node = obs_nodes
        self.clearance = clearance
        self.radius = radius
    
    def generate_graph(self):
        '''
        Returns the graph for A* and Dijkstra Algorithm
        '''
        grid_graph=[]
        # Loop through the entire grid
        for i in range(self.grid_size[0][0],self.grid_size[0][1]+1):
            for j in range(self.grid_size[1][0],self.grid_size[1][1]+1):
                # Object of class Node is created
                node=Node(i,j)
                # Checks if the node is in an Obstacle
                grid_graph.append(node)

        return grid_graph

    def get_vertices(self):
        '''
        Returns all the nodes in the graph
        '''
        #list of vertices in the graph
        return self.graph
    
    def same_node_graph(self,node):
        '''
        Returns a Node equivalent to node from graph

        Arguments:
        node-- Instance of class Node
        graph-- Free configuration Space (Instance of class Graph)

        Returns:
        eq_node-- Instance of class Node
        '''
        # loops through all the vertices in the graph
        for eq_node in self.graph:
            # Checks if two Nodes are same
            if check_nodes(eq_node,node):
                return eq_node
        return 0
    
    def checkObstacleSpace(self,node):

        (x,y) = node.get_coordinates()

        for key,values in self.obs_node.items():

            if "circle" in key:
                
                for obs in values:
                    x_centre,y_centre,rad = obs
                    if (x - x_centre)**2 + (y - y_centre)**2 <= (rad + self.radius + self.clearance)**2:

                        return True
            
            else:

                for obs in values:

                    x_min,x_max,y_min,y_max = obs

                    if x_min - self.radius - self.clearance < x <x_max +self.radius + self.clearance:

                        if y_min - self.radius - self.clearance < y <y_max +self.radius + self.clearance:

                            return True
        
        return False
