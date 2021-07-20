from graph import graph_conv,same_node_graph,check_edge_CollisionFree,Graph
from Nodes import Node,start,goal,check_nodes,check_NodeIn_list,calculate_distance
from A_star import A_star_search
from map import maze_canvas,load_map
import math
import copy
import random
import cv2
from Visualize import backtrack_list,add_path_Canvas,generate_video,draw_graph

def PRM_nbr_node(parent,nbr_list,k):
    '''
    Finds collision free neighours of parent

    Arguments:
    parent-- Object of class Node
    nbr_list-- List of nodes in the graph except the parent node
    k-- number of nearest numbers

    Returns:
    roadmap-- Dict of neighbours along with cost
    '''

    roadmap={}
    # loop through the node in the graph
    for i in nbr_list:
        # distance is calculated with nodes
        dist=calculate_distance(parent,i)
        roadmap[i]=dist
    # roadmap is sorted w.r.t to distance
    roadmap=dict(sorted(roadmap.items(), key=lambda item: item[1]))
    # k nearest neighbours are selected
    roadmap=dict(list(roadmap.items())[0:k])

    for i in list(roadmap):
        # checks if the edge between parent and nbr is collision free
        if check_edge_CollisionFree(parent,i):
            del roadmap[i]

    return roadmap

def PRM_algorithm(graph,N,k,start,goal):
    '''
    Generates graph using PRM sampling method

    Arguments:
    graph-- Free configuration Space (Instance of class Graph)
    N-- Number of nodes needed in the graph
    k-- number of neighbours for each parent node
    start-- starting node (Object of class Node)
    goal-- goal node (Object of class Node)

    Returns:
    PRM_graph-- Dict containing graph generated using PRM
    '''

    # vertices in the complete graph
    vertices=graph.get_vertices()
    # N random samples from the vertices
    random_nodes=random.sample(vertices,N)
    PRM_graph={}

    #returns an instance of start Node from the graph
    start_vertex=same_node_graph(start,graph.graph)
    goal_vertex=same_node_graph(goal,graph.graph)

    # checks if the start and goal node are in the N random samples
    if not check_NodeIn_list(start_vertex,random_nodes):
        random_nodes.append(start_vertex)
        
    if not check_NodeIn_list(goal_vertex,random_nodes):
        random_nodes.append(goal_vertex)

    # loop though all the random nodes
    for i in random_nodes:

        random_node_copy=random_nodes.copy()
        # list of nodes except the 'i' node
        random_node_copy.remove(i)

        # neighbours of 'i' node
        nbr_dict=PRM_nbr_node(i,random_node_copy,k)
        PRM_graph[i]=nbr_dict

    # if the value of any node is empty
    for pt in [key for key in PRM_graph.keys() if PRM_graph[key]=={}]:
        del PRM_graph[pt]
    PRM_graph=Graph(PRM_graph)

    return PRM_graph

def doPRM_Algorithm():

    path='PRM_Image/'
    # Make sure global variables are used
    global start,goal
    # Loads the canvas
    maze_canvas=load_map()
    # creates the graph using PRM
    PRM_graph_dict=PRM_algorithm(graph_conv,100,8,start,goal)
    # Draws the graph on the maze_canvas
    maze_canvas=draw_graph(maze_canvas,PRM_graph_dict,path)
    # Using A_star to solve the graph
    CLOSED,backtrack_node,maze_canvas=A_star_search(PRM_graph_dict,start,goal,path,maze_canvas)
    # gets the list of nodes in the shortest path
    bkt_list=backtrack_list(backtrack_node,start,goal)
    maze_canvas=add_path_Canvas(bkt_list,maze_canvas,path)
    # Generates a video
    generate_video(path)

if __name__=="__main__":
    doPRM_Algorithm()
