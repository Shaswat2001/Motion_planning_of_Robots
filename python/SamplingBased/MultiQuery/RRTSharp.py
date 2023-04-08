from Visualize import Visualize
import math
import numpy as np
import matplotlib.pyplot as plt
from heuristic import manhattan_heuristic,euclidean_heuristic
from Nodes import Node,check_NodeIn_list,check_nodes,calculate_distance
from data_structure import PriorityQueue

class RRTSharp:

    def __init__(self,start,goal,graph,tree_size = 10000,nodeDist = 3,goalDist = 1,steering_const = 1,gamma = 1):
        
        self.start = graph.same_node_graph(start)
        self.goal = graph.same_node_graph(goal)
        self.graph = graph
        self.tree_size = tree_size
        self.nodeDist = nodeDist
        self.goalDist = goalDist
        self.goal_sample_rate = 0.1
        self.steering_const = steering_const
        self.gamma = gamma

        self.visited = [self.start]
        self.g = {self.start:0}
        self.lmc = {}
        self.tree = {}
        self.queue = {}

        self.plot = Visualize(start,goal,graph.obs_boundary,graph.obs_rectangle,graph.obs_circle)

    def main(self):

        self.plan()
        node_end =self.NearestGoal()
        self.plot.animate_rrt_star("RRT#",self.visited,self.extract_path(node_end))

    def plan(self):

        for _ in range(self.tree_size):
            #  Randomly samples a node from the vertices in the graph
            sample_x=self.Sample()
            self.extend(sample_x)
            self.replan()      
    
    def extend(self,random_node):

        tree_prime = {}
        near_x=self.Nearest(self.visited,random_node)
        # new node in the tree
        new_x=self.Steer(random_node,near_x)

        if new_x not in tree_prime.keys():
            tree_prime[new_x] = []

        if not self.graph.CheckEdgeCollision(near_x,new_x):
            
            self.initialize(new_x,near_x)
            nbr_list = self.Near(new_x)

            for nbr in nbr_list:
                
                if not self.graph.CheckEdgeCollision(new_x,nbr):

                    if nbr not in tree_prime.keys():
                        tree_prime[nbr] = []

                    if self.lmc[new_x] > self.g[nbr] + calculate_distance(nbr,new_x):

                        self.lmc[new_x] = self.g[nbr] + calculate_distance(nbr,new_x)
                        new_x.parent = nbr
                    
                    tree_prime[new_x].append(nbr)
                    tree_prime[nbr].append(new_x)
            
            self.visited.append(new_x)
            self.tree_append(tree_prime)
            self.updateQueue(new_x)

    def Sample(self):

        if np.random.random() > self.goal_sample_rate:
            return self.graph.generate_random_node()

        return self.goal
                    
    def initialize(self,nbr,root):

        self.g[nbr] = math.inf

        if root:
            self.lmc[nbr] = self.g[root] + calculate_distance(root,nbr)
            nbr.parent = root
        else:
            self.lmc[nbr] = math.inf
            nbr.parent = None
    
    def tree_append(self,tree_prime):

        for key,val in tree_prime.items():

            if key in self.tree.keys():
                self.tree[key]+=val
            else:
                self.tree[key] = val
    
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

        # Checks if the distance between sampled and nearest node is less than nodeDist
        if calculate_distance(x_sampNode,x_nearNode)<self.nodeDist:
            return x_sampNode
        
        dx = x_samp[0] - x_near[0]
        dy = x_samp[1] - x_near[1]
        theta = math.atan2(dy,dx)

        x_new[0]=x_near[0] + self.nodeDist*math.cos(theta)
        x_new[1]=x_near[1] + self.nodeDist*math.sin(theta)
        newNode = Node(x_new[0],x_new[1])
        newNode.parent = x_nearNode
        # newNode = Node(round(x_new[0],2),round(x_new[1],2))
        # returns an object of class Node
        return newNode
    
    def updateQueue(self,node):

        if self.g[node] != self.lmc[node] and node in self.queue:

            self.queue[node] = self.key(node)

        elif self.g[node] != self.lmc[node] and node not in self.queue:

            self.queue[node] = self.key(node)

        elif self.g[node] == self.lmc[node] and node in self.queue:

            self.queue.pop(node)

    def key(self,node):

        return [self.lmc[node]+euclidean_heuristic(self.goal,node),self.lmc[node]]

    def check_Node_goalRadius(self,new_node):
        '''
        Checks if a Node is in the Goal radius
        '''
        if calculate_distance(self.goal,new_node) < self.goalDist:
            return True
        else:
            return False

    def Nearest(self,tree,node):
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
    
    def Near(self,new_node):

        V = len(self.visited) + 1
        radius = 50*math.sqrt(math.log(V)/V)
        # radius = min(self.gamma*math.sqrt(math.log(V)/V),self.steering_const)

        near_nbrs = [node for node in self.visited if 0<calculate_distance(node,new_node)<=radius]

        return near_nbrs
        
    def getVGoal(self):
        
        cost = {}
        for node in self.visited:
            dist=calculate_distance(node,self.goal)
            if dist < self.goalDist:
                cost[node]=self.g[node]

        if cost == {}:
            return {}
        
        cost=dict(sorted(cost.items(), key=lambda item: item[1]))
        return list(cost.keys())[0]
    
    def NearestGoal(self):

        cost = {}
        for node in self.visited:
            dist=calculate_distance(node,self.goal)
            cost[node]=dist
        
        cost=dict(sorted(cost.items(), key=lambda item: item[1]))
        return list(cost.keys())[0]
        
    def replan(self):

        if self.getVGoal() == {}:
            return 
        
        while True:

            top_node,top_key = self.TopKey()

            if top_key >= self.key(self.getVGoal()):
                break

            self.queue.pop(top_node)

            self.g[top_node] = self.lmc[top_node]

            for succ in self.tree[top_node]:

                if succ not in self.lmc.keys():
                    self.lmc[succ] = math.inf

                if self.lmc[succ] > self.g[top_node] +  calculate_distance(top_node,succ):
                    
                    self.lmc[succ] = self.g[top_node] +  calculate_distance(top_node,succ)
                    succ.parent = top_node
                    self.updateQueue(succ)

    def TopKey(self):

        s = min(self.queue, key=self.queue.get)

        return s, self.queue[s]
        
    def extract_path(self,node_end):

        bkt_list=[]
        bkt_list.append(self.goal)
        node = node_end

        while not check_nodes(node,self.start):

            bkt_list.append(node)
            node = node.parent
        
        bkt_list.append(node)

        return bkt_list