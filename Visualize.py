import matplotlib.pyplot as plt
from Nodes import check_nodes,check_NodeIn_list
from graph import cost_graph_conv,same_node_graph
from map import obstacle_points,create_obstacles
import cv2
import os
import glob

def backtrack_list(bkt_node,start,goal):
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
    bkt_list.append(goal)
    # loops till goal is not equal to zero
    while goal!=0:
        for nbr,parent in reversed(list(bkt_node.items())):
            for pt,ct in parent.items():
                # if nbr and goal are same
                if check_nodes(nbr,goal):

                    if not check_NodeIn_list(pt,bkt_list):
                        bkt_list.append(pt)

                    goal=pt

                    if check_nodes(pt,start):
                        goal=0
                        return bkt_list

def add_path_Canvas(bkt_list,canvas,path):
    '''
    Updates the canvas with the shortest path
    '''
    # Loop through the nodes in the shortest path
    for i in range(len(bkt_list)-1):
        # Coordinates of two corresponding nodes
        pt1=bkt_list[i].get_inv_coordinates()
        pt2=bkt_list[i+1].get_inv_coordinates()

        # canvas is updates
        canvas=cv2.line(canvas,pt1,pt2,(0,0,255),1,cv2.LINE_AA)
        # canvas is rotated
        flipVertical=cv2.rotate(canvas,cv2.ROTATE_90_COUNTERCLOCKWISE)
        #display flipped canvas
        cv2.imshow("MAP",flipVertical)
        # save the canvas as an image
        cv2.imwrite(f'{path}Image_st_{i}.jpg',flipVertical)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    return canvas

def draw_graph(canvas,graph,path):
    '''
    Draws complete graph on the canvas
    '''

    # the vertices in the graph
    vertices=graph.get_vertices()
    # Loop through the vertices
    for i in vertices:
        # Coordinate of 'i' node
        i_crd=i.get_inv_coordinates()
        # Loop through the neighbours of 'i' node
        for j in graph.get_neighbours(i):

            j_crd=j.get_inv_coordinates()
            # canvas is updated
            canvas=cv2.line(canvas,i_crd,j_crd,(0,255,0),1,cv2.LINE_AA)

    # canvas is flipped
    flipVertical=cv2.rotate(canvas,cv2.ROTATE_90_COUNTERCLOCKWISE)
    #display flipped canvas
    cv2.imshow("MAP",flipVertical)
    # save the canvas as an image
    cv2.imwrite(f'{path}Image_gr.jpg',flipVertical)

    return canvas

def generate_video(path):
    '''
    Generates video using the image saved
    '''
    img_array = []
    size=()
    # Loop though all the image files
    for filename in sorted(glob.glob(f'{path}/*.jpg'),key=os.path.getmtime):
        # read the image
        img = cv2.imread(filename)
        #dimension of the image
        height, width, layers = img.shape
        size = (width,height)
        img_array.append(img)

    out = cv2.VideoWriter(f'{path}/project.avi',cv2.VideoWriter_fourcc(*'DIVX'), 15,size)

    for i in range(len(img_array)):
        #image added to the video writer
        out.write(img_array[i])
        
    out.release()
