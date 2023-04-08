from data_structure import PriorityQueue
from Visualize import Visualize
from Nodes import calculate_distance,check_NodeIn_list,check_nodes
import matplotlib.pyplot as plt
import math
from heuristic import manhattan_heuristic,euclidean_heuristic


class ARAstar:
    '''
    Implements the Anytime Repairing A* algorithm for a 2D environment
    '''
    def __init__(self,start,goal,graph):

        self.start = graph.same_node_graph(start)
        self.goal = graph.same_node_graph(goal)
        self.graph = graph

        self.plot = Visualize(start,goal,graph.obstacle_points)

        self.OPEN = PriorityQueue()
        self.ICONS = []
        self.CLOSED = []
        self.past_cost = {node:math.inf for node in self.graph.get_vertices()}
        self.past_cost[self.start] = 0
        self.backtrack_node = {}
        self.epsilon = 2.5
        self.color = 0
        self.visited = []
        self.path = []

    def f_val(self,node):

        return self.past_cost[node] + self.epsilon*euclidean_heuristic(node,self.goal)

    def improvePath(self):
        self.color+=1
        while(self.f_val(self.goal) > self.OPEN.top_node()[0]):

            current_ct,current_vtx = self.OPEN.pop_pq()
            self.CLOSED.append(current_vtx)
            neighbour=self.get_neighbours(current_vtx)
            for nbr in neighbour:
                nbr_same=self.graph.same_node_graph(nbr)
                if self.past_cost[nbr_same] >self.past_cost[current_vtx] +self.cost(current_vtx,nbr_same):
                    self.past_cost[nbr_same]  = self.past_cost[current_vtx] +self.cost(current_vtx,nbr_same)
                    self.backtrack_node[nbr_same] = current_vtx
                    if check_NodeIn_list(nbr_same,self.CLOSED):
                        self.ICONS.append(nbr_same)
                    else:
                        self.OPEN.insert_pq(self.f_val(nbr_same),nbr_same)

    def main(self):

        self.OPEN.insert_pq(self.f_val(self.start),self.start)
        self.improvePath()
        self.epsilon = min(self.epsilon,self.past_cost[self.goal]/self.minVal())
        self.visited.append(self.CLOSED)
        self.path.append(self.extract_path())

        while self.epsilon > 1:
            self.epsilon -= 0.1
            print(self.epsilon)
            self.updateOPEN()
            self.CLOSED = []
            self.improvePath()
            self.visited.append(self.CLOSED)
            self.path.append(self.extract_path())
            self.epsilon = min(self.epsilon,self.past_cost[self.goal]/self.minVal())

        self.plot.animate_ara_star("Anytime Reparing A* Search",self.visited,self.path)

    def get_neighbours(self,node):

        nbrNodes = []

        nbr = node.get_neighbours()

        for node in nbr:

            if not self.graph.check_obstacleNode_canvas(node):

                nbrNodes.append(node)
        
        return nbrNodes

    def cost(self,node,parent):

        # if self.is_collision(node,parent):
        #     return math.inf
        
        return calculate_distance(node,parent)
    
    def minVal(self):

        min_ct,min_vtx = self.OPEN.top_node()
        fOPEN = self.past_cost[min_vtx]+euclidean_heuristic(min_vtx,self.goal)
        
        if len(self.ICONS) == 0:
            return fOPEN
        
        minICONS = min([self.past_cost[node]+euclidean_heuristic(node,self.goal) for node in self.ICONS])
        
        return min(minICONS,fOPEN)
    
    def updateOPEN(self):
        
        tempOPEN = PriorityQueue()

        while self.OPEN.len_pq()>0:
            min_vtx,min_ctx = self.OPEN.pop_pq()
            tempOPEN.insert_pq(self.f_val(min_ctx),min_ctx)

        for nodes in self.ICONS:
            tempOPEN.insert_pq(self.f_val(nodes),nodes)

        self.ICONS = []
        self.OPEN = tempOPEN

    def plot_visited(self):

        cl_v = ['silver',
                'wheat',
                'lightskyblue',
                'royalblue',
                'slategray']

        for nodes in self.CLOSED:
            plt.plot(nodes.x,nodes.y,marker="o",color=cl_v[self.color%len(cl_v)])
            plt.pause(0.005)

    def extract_path(self):
        '''
        Creates shortest path from start and goal node

        Returns:
        bkt_list-- List of path in the shortest path
        '''
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
