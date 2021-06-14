import matplotlib.pyplot as plt
from Nodes import check_nodes,check_NodeIn_list
from graph import cost_graph_conv,same_node_graph
from map import obstacle_points,create_obstacles
import cv2
import os
import glob

def backtrack_list(bkt_node,start,goal):
    bkt_list=[]
    bkt_list.append(goal)
    while goal!=0:
        for nbr,parent in reversed(list(bkt_node.items())):
            for pt,ct in parent.items():
                if check_nodes(nbr,goal):

                    if not check_NodeIn_list(pt,bkt_list):
                        bkt_list.append(pt)

                    goal=pt

                    if check_nodes(pt,start):
                        goal=0
                        return bkt_list

def add_path_Canvas(bkt_list,canvas,path):

    for i in range(len(bkt_list)-1):
        pt1=bkt_list[i].get_coordinates()
        pt2=bkt_list[i+1].get_coordinates()

        canvas=cv2.line(canvas,pt1,pt2,(0,0,255),2)
        flipVertical=cv2.rotate(canvas,cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow("MAP",flipVertical)
        cv2.imwrite(f'{path}Image_st_{i}.jpg',flipVertical)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    return canvas

def generate_video(path):
    img_array = []
    size=()
    for filename in sorted(glob.glob(f'{path}/*.jpg'),key=os.path.getmtime):
        img = cv2.imread(filename)
        height, width, layers = img.shape
        size = (width,height)
        img_array.append(img)

    out = cv2.VideoWriter(f'{path}/project.avi',cv2.VideoWriter_fourcc(*'DIVX'), 15,size)

    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()

def plot_graph(graph):
    global obstacle_points
    create_obstacles()
    obs_x=[]
    obs_y=[]
    for pt in obstacle_points:
        (x,y)=pt.get_coordinates()
        obs_x.append(x)
        obs_y.append(y)

    plt.scatter(obs_x,obs_y,c='blue')
    for i in graph.get_vertices():
        for j in graph.get_neighbours(i):
            i_cord=i.get_coordinates()
            j_cord=j.get_coordinates()
            plt.plot([i_cord[0],j_cord[0]],[i_cord[1],j_cord[1]],c='red')
    plt.show()

def plot_tree(tree):
    global obstacle_points
    create_obstacles()
    obs_x=[]
    obs_y=[]
    for pt in obstacle_points:
        (x,y)=pt.get_coordinates()
        obs_x.append(x)
        obs_y.append(y)

    plt.scatter(obs_x,obs_y,c='blue')
    for [i,j] in tree:
        i_cord=i.get_coordinates()
        j_cord=j.get_coordinates()
        plt.plot([i_cord[0],j_cord[0]],[i_cord[1],j_cord[1]],c='red')
    plt.show()
