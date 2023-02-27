import random
from Visualize import Visualize
from Nodes import Node,calculate_distance,check_NodeIn_list
import math
import matplotlib.pyplot as plt

class RRT:

    def __init__(self,start,goal,graph,tree_size = 10000,nodeDist = 3,goalDist = 1):

        self.start =start
        self.goal = goal
        self.graph = graph
        self.tree_size = tree_size
        self.nodeDist = nodeDist
        self.goalDist = goalDist

        self.plot = Visualize(start,goal,graph.obs_boundary,graph.obs_rectangle,graph.obs_circle)

    def main(self):

        tree,end_node = self.plan()
        self.plot.animate("RRT",tree,self.extract_path(end_node))

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

        tree=[]
        goal_reached=0

        visited=[self.start]

        # loops till size of tree is less than max_size
        while len(tree)<self.tree_size and goal_reached==0:
            
            #  Randomly samples a node from the vertices in the graph
            sample_x=self.graph.generate_random_node()
            # nearest node to sample_x
            near_x=self.nearest_node(visited,sample_x)
            # new node in the tree
            new_x=self.new_node(sample_x,near_x)

            # if path between new_node and nearest node is collision free
            if not self.graph.CheckEdgeCollision(near_x,new_x) and not check_NodeIn_list(new_x,visited):
                # add the edge to the tree
                new_x.parent = near_x
                tree.append([near_x,new_x])
                # add new node to visited list
                visited.append(new_x)

                # checks if node is in goal radius
                if self.check_Node_goalRadius(new_x):
                    print("Goal Reached")
                    goal_reached=1
                    return tree,new_x
        
        print("Goal Coudn't be reached")
        return "FAILURE",None

        
    
    def new_node(self,x_sampNode,x_nearNode):
        '''
        Generates new Node in the grid

        Arguments:
        x_sampNode-- Node sampled from the grid
        x_nearNode-- Node nearest to x_sampNode
        nodeDist-- distance between x_nearNode and new Node

        Returns:
        x_new-- Object of class Node
        '''

        x_new=[0]*2
        # Coordinates of the nodes
        x_samp=x_sampNode.get_coordinates()
        x_near=x_nearNode.get_coordinates()

        # Checks if the distance between sampled and nearest node is less than nodeDist
        if calculate_distance(x_sampNode,x_nearNode)<self.nodeDist:
            return x_sampNode
        
        dx = x_samp[0] - x_near[0]
        dy = x_samp[1] - x_near[1]
        theta = math.atan2(dy,dx)

        x_new[0]=x_near[0] + self.nodeDist*math.cos(theta)
        x_new[1]=x_near[1] + self.nodeDist*math.sin(theta)

        newNode = Node(x_new[0],x_new[1])
        # returns an object of class Node
        return newNode
    
    def check_Node_goalRadius(self,new_node):
        '''
        Checks if a Node is in the Goal radius
        '''
        if calculate_distance(self.goal,new_node) < self.goalDist:
            return True
        else:
            return False

    def nearest_node(self,tree,node):
        '''
        Finds nearest parent in the tree
        '''
        cost={}
        # Loops though all the nodes in the tree
        for i in tree:
            # distance between node and 'i'
            dist=calculate_distance(i,node)
            cost[i]=dist
        # Dict sorted with respect to distance
        cost=dict(sorted(cost.items(), key=lambda item: item[1]))
        #return closest node
        return list(cost.keys())[0]
    
    def extract_path(self,node_end):

        bkt_list=[]
        bkt_list.append(self.goal)
        node = node_end

        while node.parent != None:

            bkt_list.append(node)
            node = node.parent

        return bkt_list
