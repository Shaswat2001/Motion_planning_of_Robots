from data_structure import PriorityQueue
import math
from Visualize import Visualize
from heuristic import manhattan_heuristic,euclidean_heuristic
import matplotlib.pyplot as plt
from Nodes import check_nodes,calculate_distance,Node

class LPAstar:

    def __init__(self,start,goal,graph):

        self.start = graph.same_node_graph(start)
        self.goal = graph.same_node_graph(goal)
        self.graph = graph
        self.visited = []

        self.g = {node:math.inf for node in self.graph.get_vertices()}
        self.rhs = {node:math.inf for node in self.graph.get_vertices()}
        self.OPEN = {}
        self.rhs[self.start] = 0

        self.OPEN[self.start] = self.calculateKey(self.start)

        self.plot = Visualize(start,goal,graph.obstacle_points)
        self.plot.fig.canvas.mpl_connect('button_press_event', self.on_press)

    def main(self):

        self.computeShortestPath()
        self.plot.animate_path("Lifelong Planning A*",self.extract_path())

    def calculateKey(self,node):

        return [min(self.g[node],self.rhs[node])+euclidean_heuristic(node,self.goal),min(self.g[node],self.rhs[node])]

    def updateVertex(self,node):

        if not check_nodes(node,self.start):
            
            self.rhs[node] = min([self.g[nbNode]+self.cost(nbNode,node) for nbNode in self.Neighbours(node)])

        if node in self.OPEN:

            self.OPEN.pop(node)

        if self.g[node] != self.rhs[node]:
            self.OPEN[node] = self.calculateKey(node)

    def computeShortestPath(self):

        while True:

            current_vtx, v = self.TopKey()

            if v >= self.calculateKey(self.goal) and \
                    self.rhs[self.goal] == self.g[self.goal]:
                break

            self.OPEN.pop(current_vtx)
            self.visited.append(current_vtx)
            
            if self.g[current_vtx] > self.rhs[current_vtx]:
                self.g[current_vtx] = self.rhs[current_vtx]

            else:

                self.g[current_vtx] = math.inf
                self.updateVertex(current_vtx)

            neighbour=self.Neighbours(current_vtx)

            for nbr_node in neighbour:
                self.updateVertex(nbr_node)

    def on_press(self,event):

        x, y = event.xdata, event.ydata

        if x<0 or x>self.graph.grid_size[0] or y<0 or y>self.graph.grid_size[1]:
            print("The selected Node is outside the grid")
        else:
            
            newNode = Node(int(x),int(y))
            self.visited = []
            if self.graph.check_obstacleNode_canvas(newNode):
                print("Removing obstacle node x : ",newNode.x," y : ",newNode.y)
                self.plot.obs_map = self.graph.remove_obsNode(newNode)
                self.updateVertex(self.graph.same_node_graph(newNode))
            else:
                print("Adding obstacle node x : ",newNode.x," y : ",newNode.y)
                self.plot.obs_map = self.graph.update_obsMap(newNode)

            neighbour=self.Neighbours(newNode)
            for nbr_node in neighbour:
                self.updateVertex(nbr_node)

            self.computeShortestPath()

            plt.cla()
            self.plot.animate_lpa_star("Lifelong Planning A*",self.visited,self.extract_path())
    
    def TopKey(self):
        """
        :return: return the min key and its value.
        """

        s = min(self.OPEN, key=self.OPEN.get)

        return s, self.OPEN[s]
    
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
            neighbour=self.Neighbours(node)
            for nbr_node in neighbour:
                if not self.is_collision(node, nbr_node):
                    g_list[nbr_node] = self.g[nbr_node]
            node = min(g_list, key=g_list.get)
            bkt_list.append(node)
            if check_nodes(self.start,node):
                node = 0
                break

        return list(reversed(bkt_list))
    
    def Neighbours(self,node):

        nbrNodes = []

        nbr = node.get_neighbours()

        for node in nbr:

            if not self.graph.check_obstacleNode_canvas(node):

                nbrNodes.append(self.graph.same_node_graph(node))
        
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

