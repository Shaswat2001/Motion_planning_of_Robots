from graph import graph_conv,same_node_graph
from Nodes import Node,start,goal,check_nodes,check_NodeIn_list
from data_structure import PriorityQueue
import math

def BFS_algorithm(graph,start,goal):

    OPEN=PriorityQueue()
    CLOSED=[]
    vertices=graph.get_vertices()
    start_vertex=same_node_graph(start,graph.graph)
    past_cost={}
    past_cost[start_vertex]=0
    OPEN.insert_pq(0, start_vertex)
    goal_reached=0
    while goal_reached!=1:

        current_ct,current_vt=OPEN.pop_pq()
        CLOSED.append(current_vt)

        if check_nodes(current_vt,goal):
            goal_reached=1
            print("The goal node is found")
            return CLOSED

        neighbour=graph.get_neighbours(current_vt)
        for nbr in neighbour:

            if not check_NodeIn_list(nbr,CLOSED):

                vertex_same=same_node_graph(current_vt,graph.graph)
                nbr_same=same_node_graph(nbr,graph.graph)
                past_cost[nbr_same]=past_cost[vertex_same]+1

                OPEN.insert_pq(past_cost[nbr_same], nbr_same)
                if check_nodes(nbr_same,goal):
                    goal_reached=1
                    print("The goal node is found")
                    CLOSED.append(nbr_same)
                    return CLOSED

    print("The Goal coudnt be reached")

def doBFS():
    global start,goal
    CLOSED=BFS_algorithm(graph_conv,start,goal)
    for i in CLOSED:
        print(i.x,i.y)


if __name__=="__main__":
    doBFS()
