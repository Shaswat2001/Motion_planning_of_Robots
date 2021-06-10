from graph import graph_conv,Node_check_list,same_node_graph
from Nodes import Node,start,goal,check_nodes
from data_structure import PriorityQueue
import math

def BFS_algorithm(graph,start,goal):

    OPEN=PriorityQueue()
    CLOSED=[]
    vertices=graph.get_vertices()
    start_same=same_node_graph(start,graph.graph)
    past_cost={}
    past_cost[start_same]=0
    OPEN.insert_pq(0, start_same)
    goal_reached=0
    while goal_reached!=1:

        current_ct,current_vt=OPEN.pop_pq()
        CLOSED.append(current_vt)

        if check_nodes(current_vt,goal):
            goal_reached=1
            print("The goal node is found")
            return CLOSED,backtrack_node

        neighbour=graph.get_neighbours(current_vt)
        for nbr in neighbour:

            if not Node_check_list(nbr,CLOSED):

                vertex_same=same_node_graph(current_vt,graph.graph)
                nbr_same=same_node_graph(nbr,graph.graph)
                past_cost[nbr_same]=past_cost[vertex_same]+1

                OPEN.insert_pq(past_cost[nbr_same], nbr_same)
                if check_nodes(nbr_same,goal):
                    goal_reached=1
                    print("The goal node is found")
                    CLOSED.append(nbr_same)
                    return CLOSED,backtrack_node

    print("The Goal coudnt be reached")
