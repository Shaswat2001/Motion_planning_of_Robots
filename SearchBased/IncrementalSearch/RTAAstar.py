from Nodes import check_NodeIn_list,check_nodes,calculate_distance,Node
from Visualize import Visualize
from data_structure import PriorityQueue
import math
from heuristic import euclidean_heuristic,manhattan_heuristic
import matplotlib.pyplot as plt

class RTAAstar:

    def __init__(self,start,goal,graph,lookahead):

        self.start = graph.same_node_graph(start)
        self.goal = graph.same_node_graph(goal)
        self.graph = graph
        self.lookahead = lookahead

        self.h = {node:manhattan_heuristic(node,self.goal) for node in self.graph.get_vertices()}
        self.visited = []
        self.path = []
        self.plot = Visualize(start,goal,graph.obstacle_points)

    def main(self):

        start = self.start
        visited = []
        while True:

            OPEN, CLOSED, past_cost, backtrack = self.Astar(start)

            if OPEN == "DONE":
                sub_path = self.extract_path(start,self.goal,backtrack)
                self.path.append(sub_path)
                visited.append(self.visited)
                break

            elif OPEN.len_pq() == 0:
                print("GOAL not found")
                return
            
            s_current = OPEN.top_node()[1]
            self.update_h(CLOSED,past_cost,s_current)
            sub_path = self.extract_path(start,s_current,backtrack)
            self.path.append(sub_path)
            visited.append(self.visited)
            start = s_current
        
        self.plot.animate_rtaa_star("Real Time Adaptive A*",visited,self.path)

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
            current_cost,current_vtx = OPEN.pop_pq()
            self.visited.append(current_vtx)
            CLOSED.append(current_vtx)

            if check_nodes(current_vtx,self.goal):
                print("GOAL node is found")
                return "DONE",CLOSED,past_cost,backtrack_node


            neighbour=self.Neighbours(current_vtx)
            for nbr_node in neighbour:
                # If the neighbour is not already visited
                if not check_NodeIn_list(nbr_node,CLOSED):

                    # the tentatative_distance is calculated
                    tentatative_distance=past_cost[current_vtx]+calculate_distance(current_vtx,nbr_node)
                    # If the past_cost is greater then the tentatative_distance
                    if past_cost[nbr_node]>tentatative_distance:
                        # the neigbour node along with its parent and cost is added to the Dict
                        backtrack_node[nbr_node]=current_vtx
                        past_cost[nbr_node]=tentatative_distance
                        # Chosen heuristic is added to the tentatative_distance before adding it to the queue
                        tentatative_distance+=self.h[nbr_node]
                        # Node along with the cost is added to the queue
                        OPEN.insert_pq(tentatative_distance, nbr_node)

                        # if check_nodes(nbr_same,self.goal):
                        #     print("GOAL node is found")
                        #     return "DONE",CLOSED,past_cost,backtrack_node
            
            if count == self.lookahead:
                break
        
        return OPEN,CLOSED,past_cost,backtrack_node
    
    def Neighbours(self,node):

        nbrNodes = []

        nbr = node.get_neighbours()

        for node in nbr:

            if not self.graph.check_obstacleNode_canvas(node):

                nbrNodes.append(self.graph.same_node_graph(node))
        
        return nbrNodes
    
    def update_h(self,CLOSED,g,s_next):

        for nodes in CLOSED:

            self.h[nodes] = g[s_next] + self.h[s_next] - g[nodes]
            
    def extract_path(self,start,goal,backtrack):
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
        bkt_list.append(goal)
        node = goal
        while node!=0:
            for nbr,parent in reversed(list(backtrack.items())):
                # if nbr and goal are same
                if check_nodes(nbr,node):

                    if not check_NodeIn_list(parent,bkt_list):
                        bkt_list.append(parent)

                    node=parent

                    if check_nodes(parent,start):
                        node=0
                        return list(reversed(bkt_list))
