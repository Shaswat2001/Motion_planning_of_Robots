from Nodes import check_NodeIn_list,check_nodes,calculate_distance,Node
from Visualize import Visualize
from data_structure import PriorityQueue
import math
import matplotlib.pyplot as plt

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

class RTAAstar:

    def __init__(self,start,goal,graph,lookahead,movement):

        self.start = graph.same_node_graph(start)
        self.goal = graph.same_node_graph(goal)
        self.graph = graph
        self.lookahead = lookahead
        self.movement = movement

        self.color = 0
        self.h = {node:euclidean_heuristic(node,self.goal) for node in self.graph.get_vertices()}
        self.visited = []
        self.path = []
        self.backtrack_node = {}
        self.plot = Visualize(start,goal,graph.obstacle_points)

    def main(self):

        start = self.start
        while True:
            self.color += 1
            print(self.color)
            OPEN, CLOSED, past_cost, backtrack = self.Astar(start)
            if OPEN == "DONE":
                break

            elif OPEN.len_pq() == 0:
                print("GOAL not found")
                return
            
            top_ct,top_vtx = OPEN.top_node()
            self.update_h(CLOSED,past_cost,top_vtx)
            sub_path = self.extract_path(start,top_vtx)
            self.path.append(sub_path)
            start = top_vtx
            self.plot.plot_canvas()
            self.plot_visited()
            self.plot.shortest_path(sub_path)
        self.plot.plot_canvas()
        self.plot_visited()
        # self.plot.shortest_path(self.extract_path())
        plt.show()

    def Astar(self,start):
        self.visited = []
        OPEN = PriorityQueue()
        CLOSED = []
        backtrack_node = {}

        past_cost = {node:math.inf for node in self.graph.get_vertices()}
        past_cost[start] = 0

        OPEN.insert_pq(0,start)
        count = 0
        while OPEN.len_pq() > 0:
            count+=1
            current_ct,current_vt = OPEN.pop_pq()
            self.visited.append(current_vt)
            CLOSED.append(current_vt)

            if check_nodes(current_vt,self.goal):
                print("GOAL node is found")
                return "DONE",CLOSED,past_cost,backtrack_node


            neighbour=current_vt.get_neighbours()
            for nbr in neighbour:
                # If the neighbour is not already visited
                if not self.graph.check_obstacleNode_canvas(nbr) and not check_NodeIn_list(nbr,CLOSED):

                    # getting the same Node instance as used in cost_graph
                    vertex_same=self.graph.same_node_graph(current_vt)
                    nbr_same=self.graph.same_node_graph(nbr)

                    # the tentatative_distance is calculated
                    tentatative_distance=past_cost[vertex_same]+self.cost(vertex_same,nbr_same)
                    # If the past_cost is greater then the tentatative_distance
                    if past_cost[nbr_same]>tentatative_distance:
                        # the neigbour node along with its parent and cost is added to the Dict
                        backtrack_node[nbr_same]=vertex_same
                        past_cost[nbr_same]=tentatative_distance
                        # Chosen heuristic is added to the tentatative_distance before adding it to the queue
                        tentatative_distance+=self.h[nbr_same]
                        # Node along with the cost is added to the queue
                        OPEN.insert_pq(tentatative_distance, nbr_same)

                        # if check_nodes(nbr_same,self.goal):
                        #     print("GOAL node is found")
                        #     return "DONE",CLOSED,past_cost,backtrack_node
            
            if count == self.lookahead:
                break
        
        return OPEN,CLOSED,past_cost,backtrack_node
    
    def get_neighbours(self,node):

        nbrNodes = []

        nbr = node.get_neighbours()

        for node in nbr:

            if not self.graph.check_obstacleNode_canvas(node):

                nbrNodes.append(node)
        
        return nbrNodes
    
    def update_h(self,CLOSED,g,s_next):


        for nodes in CLOSED:
            same_node=self.graph.same_node_graph(nodes)
            self.h[same_node] = g[s_next] + self.h[s_next] - g[same_node]

    def cost(self,node,parent):

        if self.is_collision(node,parent):
            return math.inf
        
        return calculate_distance(node,parent)

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

    def plot_visited(self):

        cl_v = ['silver',
                'wheat',
                'lightskyblue',
                'royalblue',
                'slategray']

        for nodes in self.visited:
            plt.plot(nodes.x,nodes.y,marker="o",color=cl_v[self.color])
            plt.pause(0.005)

    def extract_path(self,start,goal):
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
        bkt_list.append(start)
        node = start
                    
        while True:
            h_list = {}
            neighbour=[self.graph.same_node_graph(nbr) for nbr in self.get_neighbours(node)]
            for nbr in neighbour:
                if nbr in self.h:
                    h_list[nbr] = self.h[nbr]
            s_key = max(h_list, key=h_list.get)  # move to the smallest node with min h_value
            bkt_list.append(s_key)  # generate path
            node = s_key  # use end of this iteration as the start of next

            if check_nodes(s_key,goal):  # reach the expected node in OPEN set
                return bkt_list

