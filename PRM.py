from graph import graph_conv,same_node_graph,check_edge_CollisionFree,Graph
from Nodes import Node,start,goal,check_nodes,check_NodeIn_list,calculate_distance
from A_star import A_star_search
import math
import copy
import random
from Visualize import plot_graph

def PRM_nbr_node(parent,nbr_list,k):

    roadmap={}
    for i in nbr_list:
        dist=calculate_distance(parent,i)
        roadmap[i]=dist
    roadmap=dict(sorted(roadmap.items(), key=lambda item: item[1]))

    roadmap=dict(list(roadmap.items())[0:k])
    print(roadmap)
    for i in list(roadmap):
        if check_edge_CollisionFree(parent,i):
            del roadmap[i]

    return roadmap


def PRM_algorithm(graph,N,k,start,goal):

    vertices=graph.get_vertices()
    random_nodes=random.sample(vertices,N)
    PRM_graph={}

    start_vertex=same_node_graph(start,graph.graph)
    goal_vertex=same_node_graph(goal,graph.graph)

    if not check_NodeIn_list(start_vertex,random_nodes):
        random_nodes.append(start_vertex)
    if not check_NodeIn_list(goal_vertex,random_nodes):
        random_nodes.append(goal_vertex)

    for i in random_nodes:

        random_node_copy=random_nodes.copy()
        random_node_copy.remove(i)

        nbr_dict=PRM_nbr_node(i,random_node_copy,k)
        PRM_graph[i]=nbr_dict

    for pt in [key for key in PRM_graph.keys() if PRM_graph[key]=={}]:
        del PRM_graph[pt]
    PRM_graph=Graph(PRM_graph)

    return PRM_graph

def doPRM_Algorithm(com_graph):
    global start,goal

    PRM_graph_dict=PRM_algorithm(com_graph,100,8,start,goal)
    CLOSED,backtrack_node=A_star_search(PRM_graph_dict,start,goal)

    plot_graph(PRM_graph_dict)
    for i in backtrack_node.keys():
        print(i.x,i.y)

if __name__=="__main__":
    doPRM_Algorithm(graph_conv)
