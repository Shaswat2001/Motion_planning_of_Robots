from data_structure import PriorityQueue
from Nodes import check_nodes,Node,check_NodeIn_list,calculate_distance
from Visualize import Visualize
import matplotlib.pyplot as plt
import math
class Dstar:

    def __init__(self,start,goal,graph):

        self.start = start
        self.goal = goal
        self.graph = graph
        
        self.OPEN = PriorityQueue()
        self.tags = {node:"NEW" for node in graph.get_vertices()}
        self.h = {node:math.inf for node in graph.get_vertices()}
        self.k = {node:0 for node in graph.get_vertices()}
        self.backtrack_node={node:Node(None,None) for node in graph.get_vertices()}

        self.plot = Visualize(start,goal,graph.obstacle_points)
        self.plot.fig.canvas.mpl_connect('button_press_event', self.on_press)


    def main(self):

        shortest_path = self.plan()
        self.plot.animate_path(shortest_path)

    def plan(self):

        goal_vertex = self.graph.same_node_graph(self.goal)
        start_vertex = self.graph.same_node_graph(self.start)
        self.insert(goal_vertex,0)
        while True:

            self.process_state()

            if self.tags[start_vertex] == "CLOSED":
                break
        
        return self.extract_path()
    
    def process_state(self):
        
        if self.OPEN.len_pq() == 0:
            return -1

        k_old,min_vt = self.OPEN.pop_pq()
        current_vtx = self.graph.same_node_graph(min_vt)
        self.delete(current_vtx)
        if k_old<self.h[current_vtx]:
            
            neighbour=self.get_neighbours(current_vtx)
            for nbr in neighbour:

                nbr_same=self.graph.same_node_graph(nbr)
                cost = calculate_distance(current_vtx,nbr_same)
                if self.h[nbr_same] <= k_old and self.h[current_vtx] > self.h[nbr_same] + cost:
                    self.backtrack_node[current_vtx] = nbr_same
                    self.h[current_vtx] = self.h[nbr_same] + cost

        if k_old == self.h[current_vtx]:

            neighbour=self.get_neighbours(current_vtx)
            for nbr in neighbour:

                nbr_same=self.graph.same_node_graph(nbr)
                cost = calculate_distance(current_vtx,nbr_same)
                if self.tags[nbr_same] == "NEW" or \
                    (check_nodes(self.backtrack_node[nbr_same],current_vtx) and self.h[nbr_same] != self.h[current_vtx] + cost) or \
                    (not check_nodes(self.backtrack_node[nbr_same],current_vtx) and self.h[nbr_same] > self.h[current_vtx] + cost):
                    self.backtrack_node[nbr_same] = current_vtx
                    self.insert(nbr_same,self.h[current_vtx] + cost)

        else:
            neighbour=self.get_neighbours(current_vtx)
            for nbr in neighbour:

                nbr_same=self.graph.same_node_graph(nbr)
                cost = calculate_distance(current_vtx,nbr_same)
                if self.tags[nbr_same] == "NEW" or \
                    (check_nodes(self.backtrack_node[nbr_same],current_vtx) and self.h[nbr_same] != self.h[current_vtx] + cost):
                    self.backtrack_node[nbr_same] = current_vtx
                    self.insert(nbr_same,self.h[current_vtx] + cost)

                else:

                    if not check_nodes(self.backtrack_node[nbr_same],current_vtx) and self.h[nbr_same] > self.h[current_vtx] + cost:
                        self.insert(current_vtx,self.h[current_vtx])
                    
                    elif not check_nodes(self.backtrack_node[nbr_same],current_vtx) and \
                        self.h[current_vtx] > self.h[nbr_same] + cost and \
                        self.tags[nbr_same] == "CLOSED" and self.h[nbr_same] > k_old:

                        self.insert(nbr_same,self.h[nbr_same])

        return self.get_Kmin()
    
    def on_press(self,event):

        x, y = event.xdata, event.ydata

        if x<0 or x>self.graph.grid_size[0] or y<0 or y>self.graph.grid_size[1]:
            print("The selected Node is outside the grid")
        else:

            obsNode = Node(int(x),int(y))

            if self.graph.check_obstacleNode_canvas(obsNode):
                return
            
            node = self.start
            self.visited = set()
            self.count += 1

            while not check_nodes(node,self.goal):
                if self.is_collision(s, self.PARENT[s]):
                    self.modify(s)
                    continue
                s = self.PARENT[s]


            self.plot.obs_map = self.graph.update_obsMap(obsNode)
            plt.cla()
            self.plot.plot_canvas()
            plt.show()

    def modify_state(self,X,Y,cval):
        
        pass

    def get_Kmin(self):

        if not self.OPEN:
            return -1
        
        return min([self.k[node] for node in self.graph.get_vertices()])
    
    def extract_path(self):

        bkt_list=[]
        bkt_list.append(self.start)
        node = self.start
        # loops till goal is not equal to zero
        while node!=0:
            for parent,nbr in reversed(list(self.backtrack_node.items())):
                # if nbr and goal are same
                if check_nodes(parent,node):

                    if not check_NodeIn_list(nbr,bkt_list):
                        bkt_list.append(nbr)

                    node=nbr

                    if check_nodes(nbr,self.goal):
                        node=0
                        return bkt_list

    def get_neighbours(self,node):

        nbrNodes = []

        nbr = node.get_neighbours()

        for node in nbr:

            if not self.graph.check_obstacleNode_canvas(node):

                nbrNodes.append(node)
        
        return nbrNodes
    
    def insert(self,node,h_new):

        if self.tags[node] == "NEW":
            self.k[node] = h_new

        elif self.tags[node] == "OPEN":
            self.k[node] = min(self.k[node],h_new)

        elif self.tags[node] == "CLOSED":
            self.k[node] = min(self.h[node],h_new)
        
        self.h[node] = h_new
        self.tags[node] = 'OPEN'

        self.OPEN.insert_pq(self.k[node],node)

    def delete(self,node):

        
        if self.tags[node] == "OPEN":
            self.tags[node] = "CLOSED"