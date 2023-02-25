import random
from Visualize import Visualize
from Nodes import Node,calculate_distance,check_NodeIn_list
import math
import matplotlib.pyplot as plt

class DynamicRRT:

    def __init__(self,start,goal,graph,tree_size = 20,nodeDist = 3,goalDist = 1):

        self.start = start
        self.goal = goal
        self.graph = graph
        self.tree_size = tree_size
        self.nodeDist = nodeDist
        self.goalDist = goalDist
        self.p_goal = 0.1

        self.path = []
        self.visited = [self.start]
        self.tree = []

        self.plot = Visualize(start,goal,graph.obs_boundary,graph.obs_rectangle,graph.obs_circle)
        self.plot.fig.canvas.mpl_connect('button_press_event', self.on_press)

    def main(self):

        end_node = self.plan()
        self.plot.animate("Dynamic RRT",self.tree,self.extract_path(end_node))

    def on_press(self,event):

        x, y = event.xdata, event.ydata

        if x<0 or x>self.graph.grid_size[0] or y<0 or y>self.graph.grid_size[1]:
            print("The selected Node is outside the grid")
        else:

            obsNode = [int(x),int(y)]

            if self.graph.check_obstacleCircle(obsNode):
                return
            
            print("Adding obstacle x : ",obsNode[0]," y : ",obsNode[1])
            
            self.graph.obs_circle.append([obsNode[0],obsNode[1],2])
            self.plot.obs_circle = self.graph.obs_circle
            self.visited = []
            self.invalidateNodes()
            end_node = self.regrowRRT()

            plt.cla()
            self.plot.animate("Dynamic RRT",self.tree,self.extract_path(end_node))

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
            sample_x=self.graph.generate_random_node()
            # nearest node to sample_x
            near_x=self.nearest_node(self.visited,sample_x)
            # new node in the tree
            new_x=self.new_node(sample_x,near_x)

            # if path between new_node and nearest node is collision free
            if not self.graph.check_edge_CollisionFree(near_x,new_x) and not check_NodeIn_list(new_x,self.visited):
                # add the edge to the tree
                new_x.parent = near_x
                self.tree.append([near_x,new_x])
                # add new node to visited list
                self.visited.append(new_x)

                # checks if node is in goal radius
                if self.check_Node_goalRadius(new_x):
                    print("Goal Reached")
                    return new_x
        
        return None
    
    def trimRRT(self):

        S = []
        for i in range(1,len(self.tree)):

            parent,child = self.tree[i]
            if parent.flag == "INVALID":
                child.flag = "INVALID"

            if child.flag != "INVALID":
                S.append(child)
        
        self.visited = S
        # self.visited.append(self.start)
        self.tree = [[node.parent,node] for node in S]

    def regrowRRT(self):

        self.trimRRT()
        end_node = self.plan()

        return end_node

    def invalidateNodes(self):

        for [parent,child] in self.tree:

            if self.graph.check_edge_CollisionFree(parent,child):

                child.flag = "INVALID"
    
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
    
    def generate_sample_node(self):

        p = random.random()

        if p<self.p_goal:
            return self.goal
        elif self.p_goal<p<self.p_goal+self.p_waypoint:
            return self.waypoints[random.randint(0,len(self.waypoints)-1)]
        else:
            return self.graph.generate_random_node()

