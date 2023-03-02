import random
from Visualize import Visualize
from Nodes import Node,calculate_distance,check_NodeIn_list
import math
import numpy as np
import matplotlib.pyplot as plt

class RRTStar:

    def __init__(self,start,goal,graph,tree_size = 20,nodeDist = 3,goalDist = 1,steering_const = 1,gamma = 1):

        self.start = start
        self.goal = goal
        self.graph = graph
        self.tree_size = tree_size
        self.nodeDist = steering_const
        self.goalDist = goalDist
        self.steering_const = steering_const
        self.goal_sample_rate = 0.1
        self.gamma = gamma

        self.visited = [self.start]
        self.path = []

        self.plot = Visualize(start,goal,graph.obs_boundary,graph.obs_rectangle,graph.obs_circle)

    def main(self):

        end_node = self.plan()
        self.plot.animate_rrt_star("RRT*",self.visited,self.extract_path(end_node))

    def plan(self):

        # loops till size of tree is less than max_size
        for i in range(self.tree_size):
            
            #  Randomly samples a node from the vertices in the graph
            sample_x=self.Sample()
            # nearest node to sample_x
            near_x=self.Nearest(sample_x)
            # new node in the tree
            new_x=self.Steer(sample_x,near_x)

            # if path between new_node and nearest node is collision free
            if not self.graph.CheckEdgeCollision(near_x,new_x):
                new_x.parent = near_x
                index_table = self.Near(new_x)
                self.visited.append(new_x)

                if index_table:

                    self.create_new_path(new_x,index_table)
                    self.Rewire(new_x,index_table)

        node_idx = self.connectGoal()
        return self.visited[node_idx]
    
    def Sample(self):

        if np.random.random() > self.goal_sample_rate:
                return self.graph.generate_random_node()

        return self.goal
        
    def Steer(self,x_sampNode,x_nearNode):
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

        dist = calculate_distance(x_sampNode,x_nearNode)

        dx = x_samp[0] - x_near[0]
        dy = x_samp[1] - x_near[1]
        theta = math.atan2(dy,dx)
        dist = min(self.nodeDist, dist)

        x_new[0]=x_near[0] + dist*math.cos(theta)
        x_new[1]=x_near[1] + dist*math.sin(theta)

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
        
    def connectGoal(self):
        
        dist_list = [calculate_distance(node,self.goal) for node in self.visited]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.steering_const]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(self.visited[i]) for i in node_index
                         if not self.graph.CheckEdgeCollision(self.visited[i], self.goal)]
            return node_index[int(np.argmin(cost_list))]

        return len(self.visited) - 1
        
    def create_new_path(self,new_node,distance_table):

        cost = [self.total_cost(self.visited[i],new_node) for i in distance_table]

        index = distance_table[int(np.argmin(cost))]
        new_node.parent = self.visited[index]
        
    def Rewire(self,new_node,distance_table):

        for i in distance_table:

            nbr_node = self.visited[i]

            if self.total_cost(new_node,nbr_node) < self.cost(nbr_node):
                nbr_node.parent = new_node

    def Nearest(self,node):
        '''
        Finds nearest parent in the tree
        '''
        cost={}
        # Loops though all the nodes in the tree
        for i in self.visited:
            # distance between node and 'i'
            dist=calculate_distance(i,node)
            cost[i]=dist
        # Dict sorted with respect to distance
        cost=dict(sorted(cost.items(), key=lambda item: item[1]))
        #return closest node
        return list(cost.keys())[0]
    

    def cost(self,node):

        cost = 0

        while node.parent != None:
            
            cost += calculate_distance(node,node.parent)
            node = node.parent

        return cost

    def total_cost(self,node,near_node):

        cost_node = self.cost(node)

        line_length = calculate_distance(node,near_node)

        return cost_node+line_length
        

    def Near(self,new_node):

        V = len(self.visited) + 1
        radius = min(self.gamma*math.sqrt(math.log(V)/V),self.steering_const)

        distance_table = [calculate_distance(new_node,nbr) for nbr in self.visited]
        index_dist_table = [i for i in range(len(distance_table)) if distance_table[i]<=radius and not self.graph.CheckEdgeCollision(new_node,self.visited[i])]

        return index_dist_table
    
    def extract_path(self,node_end):

        bkt_list=[]
        bkt_list.append(self.goal)
        node = node_end

        while node.parent != None:

            bkt_list.append(node)
            node = node.parent
        bkt_list.append(node)

        return bkt_list