import random
import copy
import math
from graph import check_edge_CollisionFree,same_node_graph,graph_conv
import cv2
from map import load_map
from Visualize import generate_video
from Nodes import Node,start,goal,check_nodes,check_NodeIn_list,calculate_distance

def check_Node_goalRadius(goal_node,new_node):
    '''
    Checks if a Node is in the Goal radius
    '''
    # Goal and Node Coordinates
    goal_crd=goal_node.get_coordinates()
    new_crd=new_node.get_coordinates()

    # The radius is chosen as 3 units
    if (goal_crd[0]-new_crd[0])**2 + (goal_crd[1]-new_crd[1])**2 <=(3)**2:
        return True
    else:
        return False

def nearest_node(tree,node):
    '''
    Finds nearest parent in the tree
    '''
    cost={}
    # Loops though all the nodes in the tree
    for i in tree:
        # distance between node and 'i'
        dist=calculate_distance(i,node)
        cost[i]=dist
    # Dict sorted with respect to distance
    cost=dict(sorted(cost.items(), key=lambda item: item[1]))
    #return closest node
    return list(cost.keys())[0]

def new_node(x_sampNode,x_nearNode,delta):
    '''
    Generates new Node in the grid

    Arguments:
    x_sampNode-- Node sampled from the grid
    x_nearNode-- Node nearest to x_sampNode
    delta-- distance between x_nearNode and new Node

    Returns:
    x_new-- Object of class Node
    '''

    x_new=[0]*2
    # Coordinates of the nodes
    x_samp=x_sampNode.get_coordinates()
    x_near=x_nearNode.get_coordinates()

    # Checks if the distance between sampled and nearest node is less than delta
    if calculate_distance(x_sampNode,x_nearNode)<delta:
        return x_sampNode

    # checks if the nodes lie on the line x=c
    if x_near[0]!=x_samp[0]:
        # distance between sampled and nearest node
        dist=calculate_distance(x_nearNode,x_sampNode)
        # sin and cosine angles
        costheta=(x_near[0]-x_samp[0])/dist
        sintheta=(x_near[1]-x_samp[1])/dist

        # Coordinates of the new node
        x_new[0]=int(x_near[0]-delta*costheta)
        x_new[1]=int(x_near[1]-delta*sintheta)

    else:
        x_new[0]=x_near[0]
        
        if x_near[1]>x_samp[1]:
            x_new[1]=x_near[1]+delta
        elif x_near[1]<x_samp[1]:
            x_new[1]=x_near[1]-delta
        else:
            x_new[1]=x_near[1]

    # returns an object of class Node
    return Node(*(x for x in x_new))

def RRT_algorithm(graph,start,goal,tree_size,delta,maze_canvas,path):
    '''
    Performs the RRT algorithm

    Arguments:
    graph-- Object of class Graph
    start-- starting node (Object of class Node)
    goal-- goal node (Object of class Node)
    tree_size-- max_number of edges in the tree
    delta-- distance between parent and new node
    maze_canvas-- array representing the entire grid

    returns:
    visited-- list of visited nodes
    tree-- list of edges
    '''
    video_count=0
    tree=[]
    goal_reached=0
    # vertices in the graph
    vertices=graph.get_vertices()
    # returns an instance of start Node from the graph
    start_vertex=same_node_graph(start,graph.graph)
    #returns an instance of goal Node from the graph
    goal_vertex=same_node_graph(goal,graph.graph)

    visited=[start_vertex]

    # loops till size of tree is less than max_size
    while len(tree)<tree_size and goal_reached==0:

        #  Randomly samples a node from the vertices in the graph
        sample_x=random.sample(vertices,1)[0]
        # nearest node to sample_x
        near_x=nearest_node(visited,sample_x)
        # new node in the tree
        new_x=new_node(sample_x,near_x,delta)

        # if path between new_node and nearest node is collision free
        if not check_edge_CollisionFree(near_x,new_x):
            # updates the canvas
            cv2.line(maze_canvas,new_x.get_inv_coordinates(),near_x.get_inv_coordinates(),(255,0,0),1,cv2.LINE_AA)
            # the Canvas is flipped to get the correct orientation
            flipVertical=cv2.rotate(maze_canvas,cv2.ROTATE_90_COUNTERCLOCKWISE)
            # the canvas is displayed
            cv2.imshow("MAP",flipVertical)
            # the canvas is saved as an image
            cv2.imwrite(path+f'Image_{video_count}.jpg',flipVertical)

            if cv2.waitKey(20) & 0xFF == ord('q'):
                break

            video_count+=1
            # add the edge to the tree
            tree.append([near_x,new_x])
            # add new node to visited list
            visited.append(new_x)

        # checks if node is in goal radius
        if check_Node_goalRadius(goal_vertex,new_x):
            print("Goal Reached")
            goal_reached=1

    return visited,tree

def doRRT():

    global start,goal
    path='Results/RRT_Image/'
    # Loads the canvas
    maze_canvas=load_map()
    visited,tree=RRT_algorithm(graph_conv,start,goal,2000,9,maze_canvas,path)
    # Generates a video
    generate_video(path)

if __name__=="__main__":
    doRRT()
