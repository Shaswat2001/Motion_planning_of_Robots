from Nodes import calculate_distance,check_NodeIn_list,check_nodes,Node
from Visualize import Visualize
import matplotlib.pyplot as plt
import math

class RRTConnect:

    def __init__(self,start,goal,graph,tree_size = 20,nodeDist = 3,goalDist = 1):

        self.start = start
        self.goal = goal
        self.graph = graph
        self.tree_size = tree_size
        self.nodeDist = nodeDist
        self.goalDist = goalDist

        self.treeA = []
        self.treeB = []
        self.visitedA = [self.start]
        self.visitedB = [self.goal]

        self.plot = Visualize(start,goal,graph.obs_boundary,graph.obs_rectangle,graph.obs_circle)

    def main(self):

        mid_node = self.connectPlanner()
        path = self.extract_path(mid_node)
        self.plot.animate_connect("RRT Connect",self.treeA,self.treeB,self.extract_path(mid_node))
        plt.show()
    
    def extend(self,sample,visited,tree):

        near_x=self.nearest_node(visited,sample)
            # new node in the tree
        new_x=self.new_node(sample,near_x)

        # if path between new_node and nearest node is collision free
        if not self.graph.check_edge_CollisionFree(near_x,new_x) and not check_NodeIn_list(new_x,visited):

            new_x.parent = near_x
            tree.append([near_x,new_x])
            # add new node to visited list
            visited.append(new_x)

            if check_nodes(new_x,sample):

                return "REACHED"
            
            else:
                return "ADVANCED"
        
        return "TRAPPED"



    def connect(self,sample,visited,tree):

        value = "ADVANCED"

        while value == "ADVANCED":

            value = self.extend(sample,visited,tree)
        
        return value

    
    def connectPlanner(self):

        count = 0
        for i in range(self.tree_size):
            # print(i)
            sample_x=self.graph.generate_random_node()

            if count%2 == 0:
                if self.extend(sample_x,self.visitedA,self.treeA) != "TRAPPED":

                    if self.connect(self.visitedA[-1],self.visitedB,self.treeB) == "REACHED":
                        print("Connected")
                        return self.visitedA[-1]
            
            else:
                if self.extend(sample_x,self.visitedB,self.treeB) != "TRAPPED":

                    if self.connect(self.visitedB[-1],self.visitedA,self.treeA) == "REACHED":
                        print("Connected")
                        return self.visitedB[-1]
            
            count+=1

        print("FAILED")
        return False
    
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
    
    def extract_path(self,mid_node):

        bkt_listfront=[]
        bkt_listfront.append(mid_node)
        node = mid_node

        for parent,nbr in reversed(self.treeA):
            
            if check_nodes(nbr,node):

                if not check_NodeIn_list(parent,bkt_listfront):
                    bkt_listfront.append(parent)

                    node=parent

                    if check_nodes(parent,self.start):
                        node=0
                        break

        bkt_listfront.reverse()
        
        bkt_listback=[]
        bkt_listback.append(mid_node)
        node = mid_node

        for parent,nbr in reversed(self.treeB):
            
            if check_nodes(nbr,node):

                if not check_NodeIn_list(parent,bkt_listback):
                    bkt_listback.append(parent)

                    node=parent

                    if check_nodes(parent,self.goal):
                        node=0
                        break
        
        return bkt_listfront+bkt_listback
            


