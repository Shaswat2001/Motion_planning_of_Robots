import random
from Visualize import Visualize
from Nodes import Node,calculate_distance,check_NodeIn_list,check_nodes
from heuristic import manhattan_heuristic
import math
import matplotlib.pyplot as plt
from data_structure import PriorityQueue

class LazyPRM:

    def __init__(self,start,goal,graph,numberNodes = 100,nearest_nbr = 5,max_dist = 10):

        self.start =graph.same_node_graph(start)
        self.goal = graph.same_node_graph(goal)
        self.graph = graph
        self.numberNodes = numberNodes
        self.enhancement_nodes = 10
        self.nearest_nbr = nearest_nbr
        self.max_dist = max_dist
        self.backtrack_node = {}

        self.collisionFree = False

        self.fail_connect = {}
        self.attempt_connect = {}

        self.Nodes = []
        self.grid = {}
        self.plot = Visualize(start,goal,graph.obs_boundary,graph.obs_rectangle,graph.obs_circle)

    def main(self):

        self.Nodes = self.sampleNodes()
        self.connectNodes()
        while not self.collisionFree:
            path = self.Astar()
            
            if path == None:
                print("Enhancing the Grid")
                self.nodeEnhancement()
                continue
            
            self.isPathCollisionFree(path)
            plt.cla()
            self.plot.plot_canvas("LazyPRM")
            self.plot.plot_random_nodes(self.Nodes)
            self.plot.draw_prm_grid(self.grid)
            self.plot.shortest_path(path)
        plt.show()
        # self.plot.animate_prm("LazyPRM",self.grid,self.Nodes,path)

    def sampleNodes(self,enhancement = False):

        vertices = []
        if enhancement:
            nodes = random.sample(self.graph.get_vertices(),self.enhancement_nodes//2)
        else:
            nodes = random.sample(self.graph.get_vertices(),self.numberNodes)

        for node in nodes:

            if not self.graph.check_node_CollisionFree(node):

                vertices.append(node) 

        if not check_NodeIn_list(self.goal,vertices):
            vertices.append(self.goal)

        if not check_NodeIn_list(self.start,vertices):
            vertices.append(self.start)

        return vertices
        
    def nodeEnhancement(self):

        vertices_enhancement = self.sampleNodes(enhancement=True)
        self.connectNodes(vertices_enhancement)

    def connectNodes(self,vertices = None):

        if vertices:
            grid_nodes = vertices
        else:
            grid_nodes = self.Nodes

        for node in grid_nodes:

            if node not in self.grid.keys():
                self.grid[node] = []

            nodes_copy = grid_nodes.copy()
            nodes_copy.remove(node)

            cost={}
            # Loops though all the nodes in the tree
            for i in nodes_copy:
                # distance between node and 'i'
                dist=calculate_distance(i,node)
                cost[i]=dist
            # Dict sorted with respect to distance
            cost=dict(sorted(cost.items(), key=lambda item: item[1]))

            for nbr,ct in cost.items():
                
                if nbr not in self.grid.keys():
                    self.grid[nbr] = []

                if ct < self.max_dist:

                    self.grid[node].append(nbr)
                    self.grid[nbr].append(node)

        for pt in [key for key in self.grid.keys() if self.grid[key]=={}]:
            del self.grid[pt]

    def isPathCollisionFree(self,path):

        self.collisionFree = True
        for i in range(len(path)-1):

            if self.graph.check_edge_CollisionFree(path[i],path[i+1]):
                self.collisionFree = False
                self.grid[path[i]].remove(path[i+1])
                self.grid[path[i+1]].remove(path[i])


    def Astar(self):

        vertices=self.grid.keys()

        past_cost={nodes:math.inf for nodes in vertices}
        past_cost[self.start]=0

        OPEN = PriorityQueue()
        CLOSED = []
        # Start Node with past cost is inserted into the queue
        OPEN.insert_pq(0, self.start)

        while OPEN.len_pq()>0:

            current_cost,current_vtx = OPEN.pop_pq()

            CLOSED.append(current_vtx) #Adding node to the CLOSED list

            if check_nodes(current_vtx,self.goal): # Check if goal node is reached
                print("The goal node is found")
                return self.extract_path()

            neighbour=self.grid[current_vtx]

            for nbr_node in neighbour:

                # If the neighbour is not already visited and if not in Obstacle space
                if not check_NodeIn_list(nbr_node,CLOSED):

                    # the tentatative_distance is calculated
                    tentatative_distance=past_cost[current_vtx]+calculate_distance(current_vtx,nbr_node)
                    # If the past_cost is greater then the tentatative_distance
                    if past_cost[nbr_node]>tentatative_distance:
                        # the neigbour node along with its parent is added to the Dict
                        self.backtrack_node[nbr_node]=current_vtx
                        past_cost[nbr_node]=tentatative_distance
                        # Chosen heuristic is added to the tentatative_distance before adding it to the queue
                        tentatative_distance+=manhattan_heuristic(self.goal,nbr_node)
                        # Node along with the cost is added to the queue
                        OPEN.insert_pq(tentatative_distance, nbr_node)

                        #if the goal node is reached
                        if check_nodes(nbr_node,self.goal):
                            print("The goal node is found")
                            return self.extract_path()

        # If a path Doesn't exit
        print("The Goal coudnt be reached")
        return None
    
    def extract_path(self):

        bkt_list=[]
        bkt_list.append(self.goal)
        node = self.goal
        # loops till goal is not equal to zero
        while node!=0:
            for nbr,parent in reversed(list(self.backtrack_node.items())):
                # if nbr and goal are same
                if check_nodes(nbr,node):

                    if not check_NodeIn_list(parent,bkt_list):
                        bkt_list.append(parent)

                    node=parent

                    if check_nodes(parent,self.start):
                        node=0
                        return bkt_list