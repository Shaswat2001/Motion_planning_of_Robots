from Nodes import check_NodeIn_list,calculate_distance
import math
import copy
import random

class PRM:

    def __init__(self,graph,start,goal,N=20,k=5):

        self.graph = graph
        self.start = start
        self.goal = goal
        self.N = N
        self.k = k
    
    def generate_PRM(self):
        '''
        Generates graph using PRM sampling method

        Arguments:
        graph-- Free configuration Space (Instance of class Graph)
        N-- Number of nodes needed in the graph
        k-- number of neighbours for each parent node
        start-- starting node (Object of class Node)
        goal-- goal node (Object of class Node)

        Returns:
        PRM_graph-- Dict containing graph generated using PRM
        '''

        # vertices in the complete graph
        vertices=self.graph.get_vertices()
        # N random samples from the vertices
        random_nodes=random.sample(vertices,self.N)
        PRM_graph={}

        #returns an instance of start Node from the graph
        start_vertex=self.graph.same_node_graph(self.start)
        goal_vertex=self.graph.same_node_graph(self.goal)
        # checks if the start and goal node are in the N random samples
        if not check_NodeIn_list(start_vertex,random_nodes):
            random_nodes.append(start_vertex)
            
        if not check_NodeIn_list(goal_vertex,random_nodes):
            random_nodes.append(goal_vertex)

        # loop though all the random nodes
        for i in random_nodes:

            random_node_copy=random_nodes.copy()
            # list of nodes except the 'i' node
            random_node_copy.remove(i)

            # neighbours of 'i' node
            nbr_dict=self.PRM_nbr_node(i,random_node_copy,self.k)
            PRM_graph[i]=nbr_dict

        # if the value of any node is empty
        for pt in [key for key in PRM_graph.keys() if PRM_graph[key]=={}]:
            del PRM_graph[pt]

        self.graph.graph = PRM_graph

        return self.graph
            
    def PRM_nbr_node(self,parent,nbr_list,k):
        '''
        Finds collision free neighours of parent

        Arguments:
        parent-- Object of class Node
        nbr_list-- List of nodes in the graph except the parent node
        k-- number of nearest numbers

        Returns:
        roadmap-- Dict of neighbours along with cost
        '''

        roadmap={}
        # loop through the node in the graph
        for i in nbr_list:
            # distance is calculated with nodes
            dist=calculate_distance(parent,i)
            roadmap[i]=dist
        # roadmap is sorted w.r.t to distance
        roadmap=dict(sorted(roadmap.items(), key=lambda item: item[1]))
        # k nearest neighbours are selected
        roadmap=dict(list(roadmap.items())[0:k])

        for i in list(roadmap):
            # checks if the edge between parent and nbr is collision free
            if not self.graph.check_edge_CollisionFree(parent,i):
                del roadmap[i]

        return roadmap