import random
from Visualize import Visualize
import numpy as np
from Nodes import Node,calculate_distance,check_NodeIn_list
import math
import matplotlib.pyplot as plt

class RRT:

    def __init__(self,start,goal,graph,tree_size = 10000,nodeDist = 3,goalDist = 1):

        self.start =start
        self.goal = goal
        self.graph = graph
        self.goal_sample_rate = 0.05
        self.tree_size = tree_size
        self.nodeDist = nodeDist
        self.goalDist = goalDist

        self.visited=[self.start]

        self.plot = Visualize(start,goal,graph.obs_boundary,graph.obs_rectangle,graph.obs_circle)

    def main(self):

        end_node = self.plan()
        if end_node == None:
            return
        
        self.plot.animate("RRT",self.visited,self.extract_path(end_node))

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
        for i in range(self.tree_size):
            
            #  Randomly samples a node from the vertices in the graph
            sample_x=self.Sample()
            # nearest node to sample_x
            near_x=self.Nearest(self.visited,sample_x)
            # new node in the tree
            new_x=self.Steer(sample_x,near_x)

            # if path between new node and nearest node is collision free
            if not self.graph.CheckEdgeCollision(near_x,new_x):

                new_x.parent = near_x
                # add new node to visited list
                self.visited.append(new_x)

                # checks if node is in goal radius
                if self.check_Node_goalRadius(new_x):
                    print("Goal Reached")
                    return new_x
        
        print("Goal Coudn't be reached")
        return None

    def Steer(self,x_sampNode,x_nearNode):
        '''
        Generates new Node in the grid

        Arguments:
        x_sampNode-- Node sampled from the grid
        x_nearNode-- Node nearest to x_sampNode

        Returns:
        x_new-- Object of class Node
        '''

        x_new=[0]*2
        # Coordinates of the nodes
        x_samp=x_sampNode.get_coordinates()
        x_near=x_nearNode.get_coordinates()

        dist = calculate_distance(x_sampNode,x_nearNode)        
        dx = x_samp[0] - x_near[0]
        dy = x_samp[1] - x_near[1]

        dist = min(dist,self.nodeDist)
        theta = math.atan2(dy,dx)

        x_new[0]=x_near[0] + dist*math.cos(theta)
        x_new[1]=x_near[1] + dist*math.sin(theta)

        newNode = Node(x_new[0],x_new[1])
        # newNode.parent = x_nearNode
        # returns an object of class Node
        return newNode
    
    def Sample(self):

        if np.random.random() > self.goal_sample_rate:
                return self.graph.generate_random_node()

        return self.goal
    
    def check_Node_goalRadius(self,new_node):
        '''
        Checks if a Node is in the Goal radius
        '''
        if calculate_distance(self.goal,new_node) < self.goalDist:
            return True
        else:
            return False

    @staticmethod
    def Nearest(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]
    
    def extract_path(self,node_end):

        bkt_list=[self.goal]
        node = node_end

        while node.parent != None:

            
            node = node.parent
            bkt_list.append(node)

        return bkt_list
