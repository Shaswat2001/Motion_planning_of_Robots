from Nodes import Node,calculate_distance,check_nodes,check_NodeIn_list
import numpy as np
from map import Map
import math

class Graph(Map):
    '''
    This class decribes the entire map as a graph
    '''
    def __init__(self,grid_size,delta):
        super().__init__(grid_size)

        self.graph = self.generate_cost_graph()
        self.delta = delta
    
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
    
    def generate_random_node(self):

        return Node(np.random.uniform(self.delta, self.grid_size[0] - self.delta),
                         np.random.uniform(self.delta, self.grid_size[1] - self.delta))

    def onSegment(self,p, q, r):

        if ( (q[0] <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and 
            (q[1] <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1]))):
            return True
        return False
    
    def orientation(self,p, q, r):
        # to find the orientation of an ordered triplet (p,q,r)
        # function returns the following values:
        # 0 : Collinear points
        # 1 : Clockwise points
        # 2 : Counterclockwise
        
        # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/ 
        # for details of below formula. 
        
        val = (float(q[1] - p[1]) * (r[0] - q[0])) - (float(q[0] - p[0]) * (r[1] - q[1]))
        if (val > 0):
            
            # Clockwise orientation
            return 1
        
        elif (val < 0):
            
            # Counterclockwise orientation
            return 2
        else:
            
            # Collinear orientation
            return 0

    def insideCircle(self,point,centre,radius):

        if math.sqrt((point[0]-centre[0])**2 + (point[1]-centre[1])**2) < radius:
            return True
        
        return False
    
    def isIntersect(self,p1,q1,p2,q2):
        
        o1 = self.orientation(p1, q1, p2)
        o2 = self.orientation(p1, q1, q2)
        o3 = self.orientation(p2, q2, p1)
        o4 = self.orientation(p2, q2, q1)
    
        # General case
        if ((o1 != o2) and (o3 != o4)):
            return True
    
        # Special Cases
    
        # p1 , q1 and p2 are collinear and p2 lies on segment p1q1
        if ((o1 == 0) and self.onSegment(p1, p2, q1)):
            return True
    
        # p1 , q1 and q2 are collinear and q2 lies on segment p1q1
        if ((o2 == 0) and self.onSegment(p1, q2, q1)):
            return True
    
        # p2 , q2 and p1 are collinear and p1 lies on segment p2q2
        if ((o3 == 0) and self.onSegment(p2, p1, q2)):
            return True
    
        # p2 , q2 and q1 are collinear and q1 lies on segment p2q2
        if ((o4 == 0) and self.onSegment(p2, q1, q2)):
            return True
    
        # If none of the cases
        return False


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
        parent=parent.get_coordinates()
        nbr=neighbour.get_coordinates()

        for (ox,oy,w,h) in self.obs_rectangle:
            
            coorindates = [[ox-self.delta,oy-self.delta],
                           [ox-self.delta,oy+self.delta+h],
                           [ox+w+self.delta,oy+self.delta+h],
                           [ox+w+self.delta,oy-self.delta]]
            
            if self.isIntersect(parent,nbr,coorindates[0],coorindates[1]):
                return True
            
            if self.isIntersect(parent,nbr,coorindates[1],coorindates[2]):
                return True
            
            if self.isIntersect(parent,nbr,coorindates[2],coorindates[3]):
                return True
            
            if self.isIntersect(parent,nbr,coorindates[3],coorindates[0]):
                return True
        
        for (ox,oy,w,h) in self.obs_boundary:
            
            coorindates = [[ox-self.delta,oy-self.delta],
                           [ox-self.delta,oy+self.delta+h],
                           [ox+w+self.delta,oy+self.delta+h],
                           [ox+w+self.delta,oy-self.delta]]
            
            if self.isIntersect(parent,nbr,coorindates[0],coorindates[1]):
                return True
            
            if self.isIntersect(parent,nbr,coorindates[1],coorindates[2]):
                return True
            
            if self.isIntersect(parent,nbr,coorindates[2],coorindates[3]):
                return True
            
            if self.isIntersect(parent,nbr,coorindates[3],coorindates[0]):
                return True
        
        for (ox,oy,r) in self.obs_circle:

            if self.insideCircle(parent,(ox,oy),r+self.delta):
                return True
            
            if self.insideCircle(nbr,(ox,oy),r+self.delta):
                return True

        return False

