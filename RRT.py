import random
import copy
import math
from graph import check_edge_CollisionFree,same_node_graph,graph_conv
import cv2
from map import load_map
from Nodes import Node,start,goal,check_nodes,check_NodeIn_list,calculate_distance

def check_Node_goalRadius(goal_node,new_node):
    goal_crd=goal_node.get_coordinates()
    new_crd=new_node.get_coordinates()

    if (goal_crd[0]-new_crd[0])**2 + (goal_crd[1]-new_crd[1])**2 <=(3)**2:
        return True
    else:
        return False

def nearest_node(tree,node):
    cost={}
    for i in tree:
        dist=calculate_distance(i,node)
        cost[i]=dist
    cost=dict(sorted(cost.items(), key=lambda item: item[1]))

    return list(cost.keys())[0]

def new_node(x_sampNode,x_nearNode,delta):
    x_new=[0]*2
    x_samp=x_sampNode.get_coordinates()
    x_near=x_nearNode.get_coordinates()

    if calculate_distance(x_sampNode,x_nearNode)<delta:
        return x_sampNode

    if x_near[0]!=x_samp[0]:
        dist=calculate_distance(x_nearNode,x_sampNode)
        costheta=(x_near[0]-x_samp[0])/dist
        sintheta=(x_near[1]-x_samp[1])/dist

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

    return Node(*(x for x in x_new))

def RRT_algorithm(graph,start,goal,tree_size,delta,maze_canvas):
    tree=[]
    goal_reached=0
    vertices=graph.get_vertices()
    start_vertex=same_node_graph(start,graph.graph)
    goal_vertex=same_node_graph(goal,graph.graph)
    visited=[start_vertex]

    while len(tree)<tree_size and goal_reached==0:
        sample_x=random.sample(vertices,1)[0]
        near_x=nearest_node(visited,sample_x)
        new_x=new_node(sample_x,near_x,delta)

        if not check_edge_CollisionFree(near_x,new_x):
            cv2.line(maze_canvas,new_x.get_inv_coordinates(),near_x.get_inv_coordinates(),(255,0,0),1,cv2.LINE_AA)
            flipVertical=cv2.rotate(maze_canvas,cv2.ROTATE_90_COUNTERCLOCKWISE)
            cv2.imshow("MAP",flipVertical)

            if cv2.waitKey(20) & 0xFF == ord('q'):
                break

            tree.append([near_x,new_x])
            visited.append(new_x)

        if check_Node_goalRadius(goal_vertex,new_x):
            print("Goal Reached")
            goal_reached=1

    return visited,tree

def doRRT():
    
    global start,goal
    # Loads the canvas
    maze_canvas=load_map()
    visited,tree=RRT_algorithm(graph_conv,start,goal,2000,9,maze_canvas)

if __name__=="__main__":
    doRRT()
