from Nodes import check_NodeIn_list,check_nodes,Node,calculate_distance
import matplotlib.pyplot as plt
import math
from Visualize import Visualize
class RRTStar:
    
    def __init__(self,start,goal,graph,tree_size = 20,nodeDist = 3,goalDist = 1,steering_constant = 1,gamma_rrt = 1):

        self.start = start
        self.goal = goal
        self.graph = graph
        self.tree_size = tree_size
        self.nodeDist = nodeDist
        self.goalDist = goalDist
        self.steering_constant = steering_constant
        self.gamma_rrt = gamma_rrt

        self.visited = [self.start]
        self.tree = []
        self.plot = Visualize(start,goal,graph.obs_boundary,graph.obs_rectangle,graph.obs_circle)

    
    def main(self):

        mid_node = self.connectPlanner()
        path = self.extract_path(mid_node)
        self.plot.plot_canvas()
        self.plot.draw_tree_connect(self.treeA,self.treeB)
        self.plot.shortest_path(path)
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

        for i in range(self.tree_size):

            sample=self.graph.generate_random_node()

            near_x=self.nearest_node(sample)
            # new node in the tree
            new_x=self.new_node(sample,near_x)

            if not self.graph.check_edge_CollisionFree(near_x,new_x):

                distance_table = self.find_near_neighbour(new_x)
                self.visited.append(new_x)

                if distance_table:
                    self.new_parent()
                    self.rewire()

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
    
    def find_near_neighbour(self,new_node):

        V = len(self.visited) + 1
        radius = min(self.gamma_rrt*math.sqrt(math.log(V)/V),self.steering_constant)

        dist_list = [calculate_distance(new_node,node) for node in self.visited]
        dist_index_table = [i for i in range(dist_list) if dist_list[i]<radius and not self.graph.check_edge_CollisionFree(self.visited[i],new_node)]

        return dist_index_table
            
    def nearest_node(self,node):
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
    
    def new_parent(self,new_node,index_table):

        
        cost = [self.cost(self.visited[i])+calculate_distance(self.visited[i],new_node) for i in index_table]
        
        return

    def rewire(self):

        pass
    
    def cost(self,node):

        value = 0

        while node.parent:

            value += calculate_distance(node,node.parent)
            node = node.parent
        
        return value
    
    def extract_path(self,node_end):

        bkt_list=[]
        bkt_list.append(self.goal)
        node = node_end

        while node.parent != None:

            bkt_list.append(node)
            node = node.parent

        return bkt_list

    