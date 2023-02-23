from Nodes import check_nodes,check_NodeIn_list,calculate_distance
from data_structure import PriorityQueue
import math
from Visualize import Visualize
from heuristic import manhattan_heuristic

class Astar:
    
    def __init__(self,start,goal,graph):

        self.start = start
        self.goal = goal
        self.graph = graph

        self.OPEN = PriorityQueue()
        self.CLOSED = []
        self.backtrack_node={}

        self.plot = Visualize(start,goal,graph.obstacle_points)

    def main(self):

        shortest_path,expl_nodes = self.plan()
        self.plot.animate(expl_nodes,shortest_path)

    def plan(self):
        '''
        This function implements A* Search Algorithm

        Arguments:
        start-- starting node (Instance of class Node)
        goal-- goal node (Instance of class Node)
        cost_graph-- Free configuration Space (Instance of class Graph)
        path-- Folder whe   re the images are stored
        maze_canvas-- array representing the entire grid

        Returns:
        CLOSED-- List of nodes visited by the Algorithm
        backtrack_node-- Dict used to create the shortest path
        maze_canvas-- updated array
        '''

        # vertices in the graph
        vertices=self.graph.get_vertices()
        #returns an instance of start Node from the graph
        start_vertex=self.graph.same_node_graph(self.start)

        past_cost={nodes:math.inf for nodes in vertices}
        past_cost[start_vertex]=0
        # Start Node with past cost is inserted into the queue
        self.OPEN.insert_pq(0, start_vertex)

        while self.OPEN.len_pq()>0:
        # Node with lowest past_cost is removed from the queue
            current_ct,current_vt=self.OPEN.pop_pq()
            # the Node is added to the CLOSED list
            self.CLOSED.append(current_vt)

            # if the goal node is reached
            if check_nodes(current_vt,self.goal):
                print("The goal node is found")
                return self.extract_path(),self.CLOSED

            # the neighbours of current_vt from cost_graph
            neighbour=current_vt.get_neighbours()
            for nbr in neighbour:
                # If the neighbour is not already visited
                if not self.graph.check_obstacleNode_canvas(nbr) and not check_NodeIn_list(nbr,self.CLOSED):

                    # getting the same Node instance as used in cost_graph
                    vertex_same=self.graph.same_node_graph(current_vt)
                    nbr_same=self.graph.same_node_graph(nbr)

                    # the tentatative_distance is calculated
                    tentatative_distance=past_cost[vertex_same]+calculate_distance(vertex_same,nbr_same)
                    # If the past_cost is greater then the tentatative_distance
                    if past_cost[nbr_same]>tentatative_distance:
                        # the neigbour node along with its parent and cost is added to the Dict
                        self.backtrack_node[nbr_same]=vertex_same
                        past_cost[nbr_same]=tentatative_distance
                        # Chosen heuristic is added to the tentatative_distance before adding it to the queue
                        tentatative_distance+=manhattan_heuristic(self.goal,nbr_same)
                        # Node along with the cost is added to the queue
                        self.OPEN.insert_pq(tentatative_distance, nbr_same)
                        #if the goal node is reached
                        if check_nodes(nbr_same,self.goal):
                            print("The goal node is found")
                            return self.extract_path(),self.CLOSED

                            
        # If a path Doesn't exit
        print("The Goal coudnt be reached")
        return None,self.CLOSED

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
