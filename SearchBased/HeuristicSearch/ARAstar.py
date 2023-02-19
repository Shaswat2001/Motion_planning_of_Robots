from data_structure import PriorityQueue
from Visualize import Visualize
from Nodes import calculate_distance,check_NodeIn_list,check_nodes
import matplotlib.pyplot as plt
import math

def euclidean_heuristic(node1,node2):
    '''
    This Function calculates the Euclidean distance between two nodes

    Arguments:
    node1-- Instance of class Node
    node2-- Instance of class Node

    Returns:
    euc_dist-- Euclidean distance between node1 and node2
    '''
    # Coordinates in Node1
    (x1,y1)=node1.get_coordinates()
    #Coordinates in Node2
    (x2,y2)=node2.get_coordinates()
    #The Euclidean distance rounded upto 2 decimal places
    euc_dist=round(math.sqrt((x1-x2)**2+(y1-y2)**2),2)

    return euc_dist

def manhattan_heuristic(node1,node2):
    '''
    This Function calculates the Manhattan distance between two nodes

    Arguments:
    node1-- Instance of class Node
    node2-- Instance of class Node

    Returns:
    man_dist-- Manhattan distance between node1 and node2
    '''
    # Coordinates in Node1
    (x1,y1)=node1.get_coordinates()
    #Coordinates in Node2
    (x2,y2)=node2.get_coordinates()
    #The Manhattan distance
    man_dist=abs(x1-x2)+abs(y1-y2)

    return man_dist

class ARAstar:

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
        self.plot.plot_canvas()
        self.plot_visited()
        self.plot.shortest_path(self.extract_path())
        while self.epsilon > 1:
            self.epsilon -= 0.1
            print(self.epsilon)
            self.updateOPEN()
            self.CLOSED = []
            self.improvePath()
            self.plot_visited()
            self.plot.shortest_path(self.extract_path())
            self.epsilon = min(self.epsilon,self.past_cost[self.goal]/self.minVal())

        plt.show()


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

        Arguments:
        bkt_node-- Dict containing parent and neighbour nodes
        start-- starting node (Object of class Node)
        goal-- goal node (Object of class Node)

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
