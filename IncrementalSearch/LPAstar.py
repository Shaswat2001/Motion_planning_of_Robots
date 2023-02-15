from data_structure import PriorityQueue
import math
from Visualize import Visualize
import matplotlib.pyplot as plt
from Nodes import check_nodes,check_NodeIn_list,calculate_distance,Node

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

class LPAstar:

    def __init__(self,start,goal,graph):

        self.start = graph.same_node_graph(start)
        self.goal = graph.same_node_graph(goal)
        self.graph = graph
        self.OPEN = PriorityQueue()

        self.g = {node:math.inf for node in self.graph.get_vertices()}
        self.rhs = {node:math.inf for node in self.graph.get_vertices()}

        self.rhs[self.start] = 0

        self.OPEN.insert_pq([manhattan_heuristic(self.start,self.goal),0],self.start)

        self.plot = Visualize(start,goal,graph.obstacle_points)
        self.plot.fig.canvas.mpl_connect('button_press_event', self.on_press)

    def main(self):

        self.computeShortestPath()
        self.plot.animate_path(self.extract_path())

    def calculateKey(self,node):

        return [min(self.g[node],self.rhs[node])+manhattan_heuristic(node,self.goal),min(self.g[node],self.rhs[node])]

    def updateVertex(self,node):

        if not check_nodes(node,self.start):
            
            neighbour=[self.graph.same_node_graph(nbr) for nbr in self.get_neighbours(node)]
            self.rhs[node] = min([self.g[nbNode]+self.cost(nbNode,node) for nbNode in neighbour])

        if self.OPEN.checkinPQ(node):

            self.OPEN.remove_node(node)

        if self.g[node] != self.rhs[node]:
            self.OPEN.insert_pq(self.calculateKey(node),node)

    def computeShortestPath(self):

        while self.compare_key(self.OPEN.top_node()[0],self.calculateKey(self.goal)) or self.rhs[self.goal] != self.g[self.goal]:

            min_key,min_vtx = self.OPEN.pop_pq()
            current_vtx = self.graph.same_node_graph(min_vtx)
            
            if self.g[current_vtx] > self.rhs[current_vtx]:
                self.g[current_vtx] = self.rhs[current_vtx]

            else:

                self.g[current_vtx] = math.inf
                self.updateVertex(current_vtx)

            neighbour=self.get_neighbours(current_vtx)
            for nbr in neighbour:

                nbr_same=self.graph.same_node_graph(nbr)
                self.updateVertex(nbr_same)

    def on_press(self,event):

        x, y = event.xdata, event.ydata

        if x<0 or x>self.graph.grid_size[0] or y<0 or y>self.graph.grid_size[1]:
            print("The selected Node is outside the grid")
        else:
            
            newNode = Node(int(x),int(y))

            if self.graph.check_obstacleNode_canvas(newNode):
                print("Removing obstacle node x : ",newNode.x," y : ",newNode.y)
                self.plot.obs_map = self.graph.remove_obsNode(newNode)
                self.updateVertex(self.graph.same_node_graph(newNode))
            else:
                print("Adding obstacle node x : ",newNode.x," y : ",newNode.y)
                self.plot.obs_map = self.graph.update_obsMap(newNode)

            neighbour=self.get_neighbours(newNode)
            for nbr in neighbour:
                nbr_same=self.graph.same_node_graph(nbr)  
                self.updateVertex(nbr_same)

            self.computeShortestPath()

            plt.cla()
            self.plot.plot_canvas()
            self.plot.shortest_path(self.extract_path())
            plt.show()


    def compare_key(self,k1,k2):

        if k1[0] < k2[0] or (k1[0] == k2[0] and k1[1] < k2[1]):
            return True
        
        return False
    
    def cost(self,node,parent):

        if self.is_collision(node,parent):
            return math.inf
        
        return calculate_distance(node,parent)

    def extract_path(self):

        bkt_list=[]
        bkt_list.append(self.goal)
        node = self.goal
        # loops till goal is not equal to zero

        while node != 0:
            g_list = {}
            neighbour=self.get_neighbours(node)
            for nbr in neighbour:
                nbr_same=self.graph.same_node_graph(nbr)
                g_list[nbr_same] = self.g[nbr_same]+self.cost(nbr_same,node)
            node = min(g_list, key=g_list.get)
            bkt_list.append(node)
            if check_nodes(self.start,node):
                node = 0
                break

        return list(reversed(bkt_list))
    
    def get_neighbours(self,node):

        nbrNodes = []

        nbr = node.get_neighbours()

        for node in nbr:

            if not self.graph.check_obstacleNode_canvas(node):

                nbrNodes.append(node)
        
        return nbrNodes
    
    def is_collision(self,s_start,s_end):

        if self.graph.check_obstacleNode_canvas(s_start) or self.graph.check_obstacleNode_canvas(s_end):
            return True

        if s_start.x != s_end.x and s_start.y != s_end.y:
            if s_end.x - s_start.x == s_start.y - s_end.y:
                s1 = Node(min(s_start.x, s_end.x), min(s_start.y, s_end.y))
                s2 = Node(max(s_start.x, s_end.x), max(s_start.y, s_end.y))
            else:
                s1 = Node(min(s_start.x, s_end.x), max(s_start.y, s_end.y))
                s2 = Node(max(s_start.x, s_end.x), min(s_start.y, s_end.y))

            if self.graph.check_obstacleNode_canvas(s1) or self.graph.check_obstacleNode_canvas(s2):
                return True

        return False

