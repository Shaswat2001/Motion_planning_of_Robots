from Nodes import Node,calculate_distance,check_nodes
import numpy as np
from map import Map

class Graph(Map):
    '''
    This class decribes the entire map as a graph
    '''
    def __init__(self,grid_size,obstacle_points = None,graphType = "cost"):
        super().__init__(grid_size,obstacle_points)

        if graphType == "cost":
            self.graph = self.generate_cost_graph()
        elif graphType == "uniform":
            self.graph = self.generate_graph()

    def generate_graph(self):
        '''
        Returns the graph for BFS and DFS Algorithm
        '''
        graph={}
        # Loop through the entire grid
        for i in range(self.grid_size[0]+1):
            for j in range(self.grid_size[1]+1):
                # Object of class Node is created
                node=Node(i,j)
                # Checks if the node is in an Obstacle
                if not self.check_obstacleNode_canvas(node):

                    graph[node]=[]
                    # neighbours of node
                    neighbour=list(self.neighbour_node(node).values())

                    for nbr in neighbour[0]:
                        # Object of class Node is created
                        nbr_node=Node(nbr[0],nbr[1])
                        # Checks if the neighbour is an Obstacle
                        if not self.check_obstacleNode_canvas(nbr_node):
                            # parent and neighbour is added to the graph
                            graph[node].append(nbr_node)

        return graph
    
    def neighbour_node(self,point):
        '''
        Returns a dictonary of neighbours of a particular node

        Arguments:
        point-- Instance of class Node
        '''
        # maximum x and y value of the grid
        (max_x,max_y)=self.grid_size
        # coordinates of the point
        (x,y)=point.get_coordinates()
        graph={}

        # For origin (0,0)
        if x==0 and y==0:
            graph[point]={(x+1,y),(x,y+1)}
        # For last coordinate in the grid
        elif x==max_x and y==max_y:
            graph[point]={(x-1,y),(x,y-1)}
        # For points in the x=0 and 0< y <max_y
        elif x==0 and y!=0 and y!=max_y:
            graph[point]={(x+1,y),(x,y-1),(x,y+1)}
        # For points in the y=0 and 0< x <max_x
        elif y==0 and x!=0 and x!=max_x:
            graph[point]={(x-1,y),(x+1,y),(x,y+1)}
        # For point (0,max_y)
        elif x==0 and y==max_y:
            graph[point]={(x,y-1),(x+1,y)}
        # For point (max_x,0)
        elif y==0 and x==max_x:
            graph[point]={(x-1,y),(x,y+1)}
        # For points in the y=max_y and 0< x <max_x
        elif y==max_y and x!=0 and x!=max_x:
            graph[point]={(x,y-1),(x+1,y),(x-1,y)}
        # For points in the x=max_x and 0< y <max_y
        elif x==max_x and y!=0 and y!=max_y:
            graph[point]={(x-1,y),(x,y+1),(x,y-1)}
        # For rest of the case
        else:
            graph[point]={(x+1,y),(x-1,y),(x,y+1),(x,y-1)}

        return graph
    
    def generate_cost_graph(self):
        '''
        Returns the graph for A* and Dijkstra Algorithm
        '''
        cost_graph={}
        # Loop through the entire grid
        for i in range(self.grid_size[0]+1):
            for j in range(self.grid_size[1]+1):
                # Object of class Node is created
                node=Node(i,j)
                # Checks if the node is in an Obstacle
                if not self.check_obstacleNode_canvas(node):

                    cost_graph[node]={}
                    # neighbours of node
                    neighbour=list(self.neighbour_node(node).values())

                    for nbr in neighbour[0]:
                        # Object of class Node is created
                        nbr_node=Node(nbr[0],nbr[1])
                        # Checks if the neighbour is an Obstacle
                        if not self.check_obstacleNode_canvas(nbr_node):
                            # parent and neighbour is added to the graph along with the cost
                            dist=calculate_distance(node,nbr_node)
                            cost_graph[node][nbr_node]=dist

        return cost_graph

    def get_vertices(self):
        '''
        Returns all the nodes in the graph
        '''
        #list of vertices in the graph
        vertices=list(self.graph.keys())
        return vertices

    def get_neighbours(self,node):
        '''
        Returns all the neighbours of a particular node
        '''
        # list of vertices in the graph
        vertices=list(self.graph.keys())
        for nodes in vertices:
            # if node is found in the graph
            if check_nodes(nodes,node):
                # returns an instance of node from the graph
                node_same=self.same_node_graph(node)
                return self.graph[node_same]
    
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
        for eq_node in list(self.graph.keys()):
            # Checks if two Nodes are same
            if check_nodes(eq_node,node):
                return eq_node
        return 0

def check_edge_CollisionFree(parent,neighbour):
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
    collision=False
    ot=[]
    #minimum and maximum x,y values between parent and neighbour
    min_x=min(parent[0],nbr[0])
    max_x=max(parent[0],nbr[0])
    min_y=min(parent[1],nbr[1])
    max_y=max(parent[1],nbr[1])
    # Checks if line is not of the form x=c
    if parent[0]!=nbr[0]:
        # the slope of the line
        slope=(parent[1]-nbr[1])/(parent[0]-nbr[0])
        # intermediate points are created
        for x in [min_x+(max_x-min_x)*(i/29) for i in range(30)]:
            ot.append(nbr[1]+slope*(x-nbr[0]))
    else:
        # if x coordinates of both parent and neigbour is the same
        for j in [min_y+(max_y-min_y)*(i/29) for i in range(30)]:
            ot.append(j)

    # Checks if the path is collsion free
    for x in [min_x+(max_x-min_x)*(i/29) for i in range(30)]:
        for y in ot:
            if(130+x>=y) and (290-7*x<=y) and ((17/3)*x-90<=y):
                collision=True

            if (x>=90 and 5*x-360<=y and y<=155) or (x>=90 and(x+530>=4*y) and ((5/6)*x+(170/3)<=y) and x<=130):
                collision=True

            if x>=120 and x<=160 and y>=35 and y<=130:
                if (x-10)>=y:
                    if x-400<=-2*y:
                        if 3*x-360<=y:
                            if x-60<=y or (-7/3)*x+(1120/3)>=y:
                                if (-2/5)*x +93<=y:
                                    collision=True

            if (2*x-340>=y) and ((-5/2)*x+605>=y) and (x-350>=-4*y):
                collision=True

            if (-3*x+960>=y) and ((2/11)*x+(1460/11)>=y) and ((7/2)*x-(565)>=y) and (x+580<=5*y):
                collision=True

            if collision==True:
                break

    return collision
