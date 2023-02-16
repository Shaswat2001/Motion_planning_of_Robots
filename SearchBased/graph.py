from Nodes import Node,calculate_distance,check_nodes,check_NodeIn_list
import numpy as np
from map import Map

class Graph(Map):
    '''
    This class decribes the entire map as a graph
    '''
    def __init__(self,grid_size,obstacle_points = None):
        super().__init__(grid_size,obstacle_points)

        self.graph = self.generate_cost_graph()
    
    def generate_cost_graph(self):
        '''
        Returns the graph for A* and Dijkstra Algorithm
        '''
        cost_graph=[]
        # Loop through the entire grid
        for i in range(self.grid_size[0]+1):
            for j in range(self.grid_size[1]+1):
                # Object of class Node is created
                node=Node(i,j)
                # Checks if the node is in an Obstacle
                cost_graph.append(node)

        return cost_graph

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

    def check_edge_CollisionFree(self,parent,neighbour):
        '''
        Checks if an edge between two nodes is collision Free

        Arguments:
        parent-- Object of class Node
        neighbour-- Object of class Node

        Returns:
        collision-- a boolean
        '''
        # the coordinates of parent and neigbour node
        prt=parent.get_coordinates()
        nbr=neighbour.get_coordinates()

        if check_NodeIn_list(parent,self.obstacle_points) or check_NodeIn_list(neighbour,self.obstacle_points):
            return True
        
        if prt[0] == nbr[0]:

            for nodes in self.obstacle_points:

                ndCrd = nodes.get_coordinates()

                if min(prt[1],nbr[1]) < ndCrd[1] < max(prt[1],nbr[1]) and ndCrd[0] == prt[0]:
                    return True

        else:

            slope=(prt[1]-nbr[1])/(prt[0]-nbr[0])

            for nodes in self.obstacle_points:

                ndCrd = nodes.get_coordinates()

                if ndCrd[1]-nbr[1] - slope*(ndCrd[0]-nbr[0]) == 0:
                    return True
            
        return False
