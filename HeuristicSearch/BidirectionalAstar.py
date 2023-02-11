from Astar import euclidean_heuristic,manhattan_heuristic,diagonal_heuristic
from Nodes import check_nodes,check_NodeIn_list
from data_structure import PriorityQueue
import math

class BidirectionalAstar:

    def __init__(self,start,goal,graph):

        self.start = start
        self.goal = goal
        self.graph = graph

        # Forward Planning
        self.OPEN_ford = PriorityQueue()
        self.CLOSED_ford = []
        self.backtrack_node_frd = {}

        # Backward Planning
        self.OPEN_bck = PriorityQueue()
        self.CLOSED_bck = []
        self.backtrack_node_bck = {}

    def plan(self):

        vertices=self.graph.get_vertices()
        #returns an instance of start Node from the graph
        start_vertex=self.graph.same_node_graph(self.start)
        goal_vertex = self.graph.same_node_graph(self.goal)

        past_cost_frd={nodes:math.inf for nodes in vertices}
        past_cost_frd[start_vertex]=0

        past_cost_bck={nodes:math.inf for nodes in vertices}
        past_cost_bck[goal_vertex]=0

        # Start Node with past cost is inserted into the queue
        self.OPEN_ford.insert_pq(0, start_vertex)
        self.OPEN_bck.insert_pq(0,goal_vertex)

        while self.OPEN_ford.len_pq()>0 and self.OPEN_bck.len_pq()>0:
        # Node with lowest past_cost is removed from the queue
            current_ct,current_vt=self.OPEN_ford.pop_pq()
            # the Node is added to the CLOSED list
            self.CLOSED_ford.append(current_vt)

            # if the goal node is reached
            if check_NodeIn_list(current_vt,self.goal):
                print("The goal node is found")
                return self.extract_path(),self.CLOSED

            # the neighbours of current_vt from cost_graph
            neighbour=self.graph.get_neighbours(current_vt)
            for nbr,cost in neighbour.items():
                # If the neighbour is not already visited
                if not check_NodeIn_list(nbr,self.CLOSED_ford):

                    # getting the same Node instance as used in cost_graph
                    vertex_same=self.graph.same_node_graph(current_vt)
                    nbr_same=self.graph.same_node_graph(nbr)

                    # the tentatative_distance is calculated
                    tentatative_distance=past_cost_frd[vertex_same]+cost
                    # If the past_cost is greater then the tentatative_distance
                    if past_cost_frd[nbr_same]>tentatative_distance:
                        # the neigbour node along with its parent and cost is added to the Dict
                        self.backtrack_node_frd[nbr_same]={}
                        past_cost_frd[nbr_same]=tentatative_distance
                        self.backtrack_node_frd[nbr_same][vertex_same]=tentatative_distance
                        # Chosen heuristic is added to the tentatative_distance before adding it to the queue
                        tentatative_distance+=manhattan_heuristic(self.goal,nbr_same)
                        # Node along with the cost is added to the queue
                        self.OPEN_ford.insert_pq(tentatative_distance, nbr_same)
                        #if the goal node is reached
                        if check_nodes(nbr_same,self.goal):
                            self.CLOSED_ford.append(nbr_same)
                            print("The goal node is found")
                            return self.extract_path(),self.CLOSED

                            
        # If a path Doesn't exit
        print("The Goal coudnt be reached")
        return None,self.CLOSED