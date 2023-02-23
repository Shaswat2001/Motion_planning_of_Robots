from Nodes import check_nodes,check_NodeIn_list,calculate_distance
from data_structure import PriorityQueue
import math
from heuristic import manhattan_heuristic
from Visualize import Visualize

class BidirectionalAstar:
    '''
    Implements the Bidirectional A* algorithm for a 2D environment
    '''
    def __init__(self,start,goal,graph):

        self.start = graph.same_node_graph(start)
        self.goal = graph.same_node_graph(goal)
        self.graph = graph

        # Forward Planning
        self.OPEN_ford = PriorityQueue()
        self.CLOSED_ford = []
        self.backtrack_node_frd = {}

        # Backward Planning
        self.OPEN_bck = PriorityQueue()
        self.CLOSED_bck = []
        self.backtrack_node_bck = {}

        self.plot = Visualize(start,goal,graph.obstacle_points)

    def main(self):

        shortest_path,expl_nodes_frd,expl_nodes_bck = self.plan()
        self.plot.animate_bi("Bidirectional A* Search",expl_nodes_frd,expl_nodes_bck,shortest_path)

    def plan(self):
        '''
        Returns:
        path -- the shortest path between start and goal node
        CLOSED -- List of nodes visited by the Algorithm in forward and backward pass
        '''
        vertices=self.graph.get_vertices()

        # past cost and OPEN queue for forward pass
        past_cost_frd={nodes:math.inf for nodes in vertices}
        past_cost_frd[self.start]=0

        self.OPEN_ford.insert_pq(0, self.start)

        # past cost and OPEN queue for backward pass
        past_cost_bck={nodes:math.inf for nodes in vertices}
        past_cost_bck[self.goal]=0
        
        self.OPEN_bck.insert_pq(0,self.goal)

        # ----- FORWARD PASS -----
        while self.OPEN_ford.len_pq()>0 and self.OPEN_bck.len_pq()>0:

            current_cost,current_vtx=self.OPEN_ford.pop_pq()

            if check_NodeIn_list(current_vtx,self.backtrack_node_bck.keys()):
                print("The goal node is found")
                return self.extract_path(current_vtx),self.CLOSED_ford,self.CLOSED_bck
            
            self.CLOSED_ford.append(current_vtx)

            neighbour=self.get_neighbours(current_vtx)
            for nbr_node in neighbour:

                if not check_NodeIn_list(nbr_node,self.CLOSED_ford):

                    # the tentatative_distance is calculated
                    tentatative_distance=past_cost_frd[current_vtx]+calculate_distance(current_vtx,nbr_node)
                    # If the past_cost is greater then the tentatative_distance
                    if past_cost_frd[nbr_node]>tentatative_distance:
                        # the neigbour node along with its parent is added to the Dict
                        self.backtrack_node_frd[nbr_node]=current_vtx
                        past_cost_frd[nbr_node]=tentatative_distance
                        # Chosen heuristic is added to the tentatative_distance before adding it to the queue
                        tentatative_distance+=manhattan_heuristic(self.goal,nbr_node)
                        # Node along with the cost is added to the queue
                        self.OPEN_ford.insert_pq(tentatative_distance, nbr_node)
            
            #  ----- BACKWARD PASS -----
            backward_cost,backward_vtx=self.OPEN_bck.pop_pq()

            if check_NodeIn_list(backward_vtx,self.backtrack_node_frd.keys()):
                print("The goal node is found")
                return self.extract_path(backward_vtx),self.CLOSED_ford,self.CLOSED_bck
            
            self.CLOSED_bck.append(backward_vtx)

            neighbour=self.get_neighbours(backward_vtx)
            for nbr_back in neighbour:

                if not check_NodeIn_list(nbr_back,self.CLOSED_bck):

                    # the tentatative_distance is calculated
                    tentatative_distance=past_cost_bck[backward_vtx]+calculate_distance(backward_vtx,nbr_back)
                    # If the past_cost is greater then the tentatative_distance
                    if past_cost_bck[nbr_back]>tentatative_distance:
                        # the neigbour node along with its parent and cost is added to the Dict
                        self.backtrack_node_bck[nbr_back]=backward_vtx
                        past_cost_bck[nbr_back]=tentatative_distance
                        # Chosen heuristic is added to the tentatative_distance before adding it to the queue
                        tentatative_distance+=manhattan_heuristic(self.start,nbr_back)
                        # Node along with the cost is added to the queue
                        self.OPEN_bck.insert_pq(tentatative_distance, nbr_back)
                            
        # If a path Doesn't exit
        print("The Goal coudnt be reached")
        return None,self.CLOSED_ford,self.CLOSED_bck
    
    def get_neighbours(self,current_node):

        nbr_list = []
        neighbour=current_node.get_neighbours()

        for nbr in neighbour:
            # If the neighbour is not in Obstacle space
            if not self.graph.check_obstacleNode_canvas(nbr):

                nbr_list.append(self.graph.same_node_graph(nbr))

        return nbr_list
    
    def extract_path(self,meetNode):

        bkt_list=[]
        bkt_list.append(meetNode)
        node = meetNode # The node where forward and backward pass converged

        # loops till goal is not equal to zero
        while node!=0:
            for nbr,parent in reversed(list(self.backtrack_node_bck.items())):
                # if nbr and goal are same
                if check_nodes(nbr,node):

                    if not check_NodeIn_list(parent,bkt_list):
                        bkt_list.append(parent)

                    node=parent

                    if check_nodes(parent,self.goal):
                        node=0
                        break
                    
        bkt_list.reverse()
        node = meetNode

        while node!=0:
            for nbr,parent in reversed(list(self.backtrack_node_frd.items())):
                # if nbr and goal are same
                if check_nodes(nbr,node):

                    if not check_NodeIn_list(parent,bkt_list):
                        bkt_list.append(parent)

                    node=parent

                    if check_nodes(parent,self.start):
                        node=0
                        return bkt_list