import random
import copy
import math
from graph import check_edge_CollisionFree,same_node_graph,graph_conv
from A_star import A_star_search
from Visualize import plot_graph
from Nodes import Node,start,goal,check_nodes,check_NodeIn_list,calculate_distance

def nearest_node(tree,node):

    cost={}
    for i in tree:
        dist=calculate_distance(i,node)
        cost[i]=dist
    cost=dict(sorted(cost.items(), key=lambda item: item[1]))

    return cost.keys()[0]

def new_Node(nd_near,nd_samp,delta):
    x_new=[0,0]
    x_near=nd_near.get_coordinates()
    x_samp=nd_samp.get_coordinates()
    x_new=[0]*2
    if x_samp[0]!=x_near[0]:
        slope=abs((x_samp[1]-x_near[1])/(x_samp[0]-x_near[0]))
        costheta=1/math.sqrt(1+slope**2)
        sintheta=slope/math.sqrt(1+slope**2)
        y_disp=int(delta*sintheta)+1
        x_disp=int(delta*costheta)+1

        if x_near[0]>x_samp[0]:
            x_new[0]=x_near[0]-x_disp
        elif x_near[0]<x_samp[0]:
            x_new[0]=x_near[0]+x_disp

        if x_near[1]>x_samp[1]:
            x_new[1]=x_near[1]-y_disp
        elif x_near[1]<x_samp[1]:
            x_new[1]=x_near[1]+y_disp
        else:
            x_new[1]=x_near[1]

    else:
        x_new[0]=x_near[0]
        if x_near[1]>x_samp[1]:
            x_new[1]=x_near[1]-delta
        elif x_near[1]<x_samp[1]:
            x_new[1]=x_near[1]+delta
        else:
            x_new[1]=x_near[1]

    return Node(*(x for x in x_new))

def RRT(graph,start):
    tree=[]
    start_vertex=same_node_graph(start,graph.graph)
    goal_vertex=same_node_graph(goal,graph.graph)
    visited=[start_vertex]
    vertices=graph.get_vertices()
    while len(tree)<max_trees:
        samp_pt=random.sample(vertices,1)[0]
        near_x=nearest_node(visited,samp_pt)
        x_new=new_Node(near_x,samp_pt,delta)
        x_new_vertex=same_node_graph(x_new,graph.graph)
        path=check_edge_CollisionFree(x_new_vertex,near_x)
        if not path:
            x_new_vertex.parent=near_x
            x_new_vertex.cost=calculate_distance(x_new_vertex,near_x)
            tree.append((near_x,x_new_vertex))
            visited.append(x_new_vertex)

    return tree,visited

def doRRT_Algorithm(com_graph):
    global start,goal

    tree,visited=PRM_algorithm(com_graph,100,8,start,goal)

    plot_graph(PRM_graph_dict)


if __name__=="__main__":
    doPRM_Algorithm(graph_conv)
