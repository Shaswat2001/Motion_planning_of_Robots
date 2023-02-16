import random
from Visualize import Visualize
from Nodes import Node,calculate_distance
import matplotlib.pyplot as plt

class RRT:

    def __init__(self,start,goal,graph,tree_size = 20,delta = 3):

        self.start = start
        self.goal = goal
        self.graph = graph
        self.tree_size = tree_size
        self.delta = delta

        self.plot = Visualize(start,goal,graph.obs_boundary,graph.obs_rectangle,graph.obs_circle)

    def main(self):

        # visited,tree = self.plan()
        self.plot.plot_canvas()
        plt.show()
        # self.plot.draw_tree(tree)

    def plan(self):
        '''
        Performs the RRT algorithm

        Arguments:
        graph-- Object of class Graph
        start-- starting node (Object of class Node)
        goal-- goal node (Object of class Node)
        tree_size-- max_number of edges in the tree
        delta-- distance between parent and new node
        maze_canvas-- array representing the entire grid

        returns:
        visited-- list of visited nodes
        tree-- list of edges
        '''

        tree=[]
        goal_reached=0
        # vertices in the graph
        vertices=self.graph.get_vertices()
        # returns an instance of start Node from the graph
        start_vertex=self.graph.same_node_graph(self.start)
        #returns an instance of goal Node from the graph
        goal_vertex=self.graph.same_node_graph(self.goal)

        visited=[start_vertex]

        # loops till size of tree is less than max_size
        while len(tree)<self.tree_size and goal_reached==0:
            
            #  Randomly samples a node from the vertices in the graph
            sample_x=random.sample(vertices,1)[0]
            # nearest node to sample_x
            near_x=self.nearest_node(visited,sample_x)
            # new node in the tree
            new_x=self.new_node(sample_x,near_x)

            # if path between new_node and nearest node is collision free
            if not self.graph.check_edge_CollisionFree(near_x,new_x):
                # add the edge to the tree
                tree.append([near_x,new_x])
                # add new node to visited list
                visited.append(new_x)

                # checks if node is in goal radius
                if self.check_Node_goalRadius(new_x):
                    print("Goal Reached")
                    goal_reached=1

        return visited,tree
    

    def new_node(self,x_sampNode,x_nearNode):
        '''
        Generates new Node in the grid

        Arguments:
        x_sampNode-- Node sampled from the grid
        x_nearNode-- Node nearest to x_sampNode
        delta-- distance between x_nearNode and new Node

        Returns:
        x_new-- Object of class Node
        '''

        x_new=[0]*2
        # Coordinates of the nodes
        x_samp=x_sampNode.get_coordinates()
        x_near=x_nearNode.get_coordinates()

        # Checks if the distance between sampled and nearest node is less than delta
        if calculate_distance(x_sampNode,x_nearNode)<self.delta:
            return x_sampNode

        # checks if the nodes lie on the line x=c
        if x_near[0]!=x_samp[0]:
            # distance between sampled and nearest node
            dist=calculate_distance(x_nearNode,x_sampNode)
            # sin and cosine angles
            costheta=(x_near[0]-x_samp[0])/dist
            sintheta=(x_near[1]-x_samp[1])/dist

            # Coordinates of the new node
            x_new[0]=int(x_near[0]-self.delta*costheta)
            x_new[1]=int(x_near[1]-self.delta*sintheta)

        else:
            x_new[0]=x_near[0]
            
            if x_near[1]>x_samp[1]:
                x_new[1]=x_near[1]+self.delta
            elif x_near[1]<x_samp[1]:
                x_new[1]=x_near[1]-self.delta
            else:
                x_new[1]=x_near[1]

        # returns an object of class Node
        return Node(*(x for x in x_new))
    

    def check_Node_goalRadius(self,new_node):
        '''
        Checks if a Node is in the Goal radius
        '''
        # Goal and Node Coordinates
        goal_crd=self.goal.get_coordinates()
        new_crd=new_node.get_coordinates()

        # The radius is chosen as 1 unit
        if (goal_crd[0]-new_crd[0])**2 + (goal_crd[1]-new_crd[1])**2 <=(3)**2:
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
