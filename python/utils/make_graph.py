import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.spatial.transform import Rotation as Rot
from utils.Nodes import check_nodes
from utils.non_holonomic import NonHolonomicDrive
import math
import numpy as np
import os
import glob

class Visualize:

    def __init__(self,start,goal,obs_node,RPM,grid_size):
        
        self.start = start
        self.goal = goal
        self.obs_node = obs_node
        self.drive = NonHolonomicDrive(RPM,grid_size,send_int_nodes=True)

        self.fig, self.ax = plt.subplots()
    
    def animate(self,algorithm,visited,path):
        self.plot_canvas(algorithm)
        # self.plot_visited(visited)
        self.shortest_path(path)
        plt.show()

    def animate_connect(self,algorithm,treeA,treeB,path):
        self.plot_canvas(algorithm)
        self.draw_tree_connect(treeA,treeB)
        self.shortest_path(path)
        plt.show()

    def plot_canvas(self,algorithm):
        
        for key,values in self.obs_node.items():

            if "circle" in key:
                for (ox,oy,r) in values:
                    self.ax.add_patch(
                        patches.Circle(
                            (ox, oy), r,
                            edgecolor='black',
                            facecolor='gray',
                            fill=True
                        )
                    )
                    
            elif "boundary" in key:
                for (x_min,x_max,y_min,y_max) in values:
                    w = abs(x_max - x_min)
                    h = abs(y_max - y_min)
                    self.ax.add_patch(
                        patches.Rectangle(
                            (x_min, y_min), w, h,
                            edgecolor='black',
                            facecolor='black',
                            fill=True
                        )
                    )

            else:
                for (x_min,x_max,y_min,y_max) in values:
                    w = abs(x_max - x_min)
                    h = abs(y_max - y_min)
                    self.ax.add_patch(
                        patches.Rectangle(
                            (x_min, y_min), w, h,
                            edgecolor='black',
                            facecolor='gray',
                            fill=True
                        )
                    )

        plt.scatter(self.start.x,self.start.y,color="magenta")
        plt.scatter(self.goal.x,self.goal.y,color="blue")
        plt.axis("equal")
        plt.title(algorithm)

    def plot_random_nodes(self,node_list):

        for node in node_list:

            plt.scatter(node.x,node.y,color="lightgrey",s=2)

    def draw_tree(self,tree):

        # Loop through the vertices
        for prt,node in tree:
            # Coordinate of 'i' node
            root=prt.get_coordinates()
            nbr=node.get_coordinates()
            plt.plot([root[0],nbr[0]],[root[1],nbr[1]],'-g')
            # plt.pause(0.01)
    
    def draw_prm_grid(self,grid):

        for parent,nbr_list in grid.items():
            parent = parent.get_coordinates()
            for nbr_node in nbr_list:
                nbr_node = nbr_node.get_coordinates()
                plt.plot([parent[0],nbr_node[0]],[parent[1],nbr_node[1]],linewidth='1', color="darkgreen")

    def draw_tree_connect(self,treeA,treeB):

        for i in range(min(len(treeA),len(treeB))):

            rootA=treeA[i][0].get_coordinates()
            nbrA=treeA[i][1].get_coordinates()

            plt.plot([rootA[0],nbrA[0]],[rootA[1],nbrA[1]],linewidth='1', color="darkgreen")

            rootB=treeB[i][0].get_coordinates()
            nbrB=treeB[i][1].get_coordinates()

            plt.plot([rootB[0],nbrB[0]],[rootB[1],nbrB[1]],linewidth='1', color="darkgreen")
            plt.pause(0.01)

        while i<=len(treeA)-1:

            root=treeA[i][0].get_coordinates()
            nbr=treeA[i][1].get_coordinates()
            plt.plot([root[0],nbr[0]],[root[1],nbr[1]],linewidth='1', color="darkgreen")
            plt.pause(0.01)
            i+=1

        while i<=len(treeB)-1:

            root=treeB[i][0].get_coordinates()
            nbr=treeB[i][1].get_coordinates()
            plt.plot([root[0],nbr[0]],[root[1],nbr[1]],linewidth='1', color="darkgreen")
            plt.pause(0.01)
            i+=1

    def shortest_path(self,path):

        for i in reversed(range(1,len(path))):
            start = path[i]
            end = path[i-1]

            theta = np.rad2deg(math.atan2(end.y-start.y,end.x-start.x))%360
            neighbours = self.drive.get_neighbours(start,theta)

            for key,value in neighbours.items():

                (_,_,int_nodes) = value

                int_node_x = []
                int_node_y = []
                for i in range(len(int_nodes)):
                    int_node_x.append(round(int_nodes[i][0],2))
                    int_node_y.append(round(int_nodes[i][1],2))
                plt.plot(int_node_x, int_node_y,color="r")

        plt.scatter(self.start.x,self.start.y,color="magenta")
        plt.scatter(self.goal.x,self.goal.y,color="blue")
        plt.pause(0.01)


        