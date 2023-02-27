import random
from Visualize import Visualize
from Nodes import Node,calculate_distance,check_NodeIn_list
import math
import numpy as np
import matplotlib.pyplot as plt

class RRTStarSmart:

    def __init__(self,start,goal,graph,tree_size = 20,nodeDist = 3,goalDist = 1,steering_const = 1,gamma = 1):

        self.start = start
        self.goal = goal
        self.graph = graph
        self.tree_size = tree_size
        self.nodeDist = nodeDist
        self.goalDist = goalDist
        self.steering_const = steering_const
        self.gamma = gamma

        self.initialPathFound = False
        self.beacon_nodes = []
        self.b = 2
        self.n = 0
        self.beacons_radius = 2
        self.direct_cost_old = math.inf
        self.goal_sample_rate = 0.1
        self.obs_vertex = self.graph.get_obs_vertex()

        self.visited = [self.start]
        self.path = []

        self.plot = Visualize(start,goal,graph.obs_boundary,graph.obs_rectangle,graph.obs_circle)

    def main(self):

        self.plan()
        self.plot.animate_rrt_star("RRT* Smart",self.visited,self.extract_path())

    def plan(self):
        
        # loops till size of tree is less than max_size
        for i in range(self.tree_size):

            if (i-self.n)%self.b == 0 and len(self.beacon_nodes) > 0:
                sample_x = self.Sample(self.beacon_nodes)
            else:
                sample_x=self.Sample()
                
            # nearest node to sample_x
            near_x=self.Nearest(self.visited,sample_x)
            # new node in the tree
            new_x=self.Steer(sample_x,near_x)

            # if path between new_node and nearest node is collision free
            if not self.graph.check_edge_CollisionFree(near_x,new_x):

                index_table = self.Near(new_x)

                self.visited.append(new_x)

                if index_table:

                    self.create_new_path(new_x,index_table)
                    self.rewire(new_x,index_table)
                
                if not self.initialPathFound and self.check_Node_goalRadius(new_x):

                    self.initialPathFound = True
                    self.n = i

                if self.initialPathFound:

                    self.PathOptimization(new_x)

        node_idx = self.connectGoal()
        return self.visited[node_idx]
    
    def Sample(self, goal=None):

        if goal is None:
            goal_sample_rate = self.goal_sample_rate

            if np.random.random() > goal_sample_rate:
                return self.graph.generate_random_node()

            return self.goal
        else:
            R = self.beacons_radius
            r = random.uniform(0, R)
            theta = random.uniform(0, 2 * math.pi)
            ind = random.randint(0, len(goal) - 1)

            return Node(goal[ind][0] + r * math.cos(theta),
                         goal[ind][1] + r * math.sin(theta))

    
    def PathOptimization(self,node):

        direct_cost_new = 0.0
        node_end = self.goal

        while node.parent:
            node_parent = node.parent
            if not self.graph.check_edge_CollisionFree(node_parent, node_end):
                node_end.parent = node_parent
            else:
                direct_cost_new += calculate_distance(node, node_end)
                node_end = node

            node = node_parent

        if direct_cost_new < self.direct_cost_old:
            self.direct_cost_old = direct_cost_new
            self.UpdateBeacons()

    def UpdateBeacons(self):
        node = self.goal
        beacons = []

        while node.parent:
            near_vertex = [v for v in self.obs_vertex
                           if (node.x - v[0]) ** 2 + (node.y - v[1]) ** 2 < 9]
            if len(near_vertex) > 0:
                for v in near_vertex:
                    beacons.append(v)

            node = node.parent

        self.beacon_nodes = beacons
        
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
                         if not self.graph.check_edge_CollisionFree(self.visited[i], self.goal)]
            return node_index[int(np.argmin(cost_list))]

        return len(self.visited) - 1
        
    def create_new_path(self,new_node,distance_table):

        cost = [self.total_cost(self.visited[i],new_node) for i in distance_table]

        index = distance_table[int(np.argmin(cost))]
        new_node.parent = self.visited[index]
        
    def rewire(self,new_node,distance_table):

        for i in distance_table:

            nbr_node = self.visited[i]

            if self.total_cost(new_node,nbr_node) < self.cost(nbr_node):
                nbr_node.parent = new_node

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
        index_dist_table = [i for i in range(len(distance_table)) if distance_table[i]<=radius and not self.graph.check_edge_CollisionFree(new_node,self.visited[i])]

        return index_dist_table
    
    def extract_path(self):

        bkt_list=[self.goal]
        node = self.goal

        while node.parent != None:

            bkt_list.append(node)
            node = node.parent
        bkt_list.append(node)

        return bkt_list