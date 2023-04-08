from Nodes import check_nodes,check_NodeIn_list,calculate_distance
from data_structure import PriorityQueue
import math
from Visualize import Visualize


class Dijkstra:
    '''
    Implements the Dijkstra algorithm for a 2D environment
    '''
    def __init__(self,start,goal,graph):

        self.start = graph.same_node_graph(start)
        self.goal = graph.same_node_graph(goal)
        self.graph = graph

        self.OPEN = PriorityQueue()
        self.CLOSED = []
        self.backtrack_node={}
        self.plot = Visualize(start,goal,graph.obstacle_points)

    def main(self):

        shortest_path,expl_nodes = self.plan()
        self.plot.animate("Dijkstra Search",expl_nodes,shortest_path)

    def plan(self):
        '''
        Finds the optimal path

        Returns:
        path -- the shortest path between start and goal node
        CLOSED -- List of nodes visited by the Algorithm
        '''

        # vertices in the graph
        vertices=self.graph.get_vertices()

        past_cost={nodes:math.inf for nodes in vertices}
        past_cost[self.start]=0

        self.OPEN.insert_pq(0, self.start)

        while self.OPEN.len_pq()>0:

            current_cost,current_vtx=self.OPEN.pop_pq()

            self.CLOSED.append(current_vtx)

            # if the goal node is reached
            if check_nodes(current_vtx,self.goal):
                print("The goal node is found")
                return self.extract_path(),self.CLOSED

            neighbour=self.get_neighbours(current_vtx)
            for nbr_node in neighbour:
                
                # If the neighbour is not already visited
                if not check_NodeIn_list(nbr_node,self.CLOSED):

                    # the tentatative_distance is calculated
                    tentatative_distance=past_cost[current_vtx]+calculate_distance(current_vtx,nbr_node)
                    # If the past_cost is greater then the tentatative_distance
                    if past_cost[nbr_node]>tentatative_distance:
                        # the neigbour node along with its parent and cost is added to the Dict
                        self.backtrack_node[nbr_node]=current_vtx
                        past_cost[nbr_node]=tentatative_distance
                        # Node along with the cost is added to the queue
                        self.OPEN.insert_pq(tentatative_distance, nbr_node)
                        #if the goal node is reached
                        if check_nodes(nbr_node,self.goal):
                            print("The goal node is found")
                            return self.extract_path(),self.CLOSED

        # If a path Doesn't exit
        print("The Goal coudnt be reached")
        return None,self.CLOSED
    
    def get_neighbours(self,current_node):

        nbr_list = []
        neighbour=current_node.get_neighbours()

        for nbr in neighbour:
            # If the neighbour is not in Obstacle space
            if not self.graph.check_obstacleNode_canvas(nbr):

                nbr_list.append(self.graph.same_node_graph(nbr))

        return nbr_list

    def extract_path(self):
        '''
        Creates shortest path from start and goal node

        Returns:
        bkt_list -- List of nodes in the shortest path
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