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

        self.drive = NonHolonomicDrive(RPM,grid_size)

        self.fig, self.ax = plt.subplots()
    
    def animate(self,algorithm,path):
        self.plot_canvas(algorithm)
        self.plot_visited(path)
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

    def shortest_path(self,path):

        for i in reversed(range(1,len(path))):
            start = path[i]["vertex"]
            end = path[i-1]["vertex"]
            theta = path[i]["orientation"]
            # theta = np.rad2deg(math.atan2(end.y-start.y,end.x-start.x))%360
            int_node_x = []
            int_node_y = []
            for x,y in path[i]["intermediate_nodes"]:
    
                int_node_x.append(round(x,2))
                int_node_y.append(round(y,2))
            plt.plot(int_node_x, int_node_y,color="r")
            plt.pause(0.01)

        plt.scatter(self.start.x,self.start.y,color="magenta")
        plt.scatter(self.goal.x,self.goal.y,color="blue")
        plt.pause(0.01)

    def plot_visited(self,path):

        for i in reversed(range(1,len(path))):
            start = path[i]["vertex"]
            end = path[i-1]["vertex"]
            theta = path[i]["orientation"]

            neighbours = self.drive.get_neighbours(start,theta)
            # theta = np.rad2deg(math.atan2(end.y-start.y,end.x-start.x))%360
            for _,value in neighbours.items():

                int_nodes = value["intermediate_nodes"]

                int_node_x = []
                int_node_y = []
                for i in range(len(int_nodes)):
                    int_node_x.append(round(int_nodes[i][0],2))
                    int_node_y.append(round(int_nodes[i][1],2))
                plt.plot(int_node_x, int_node_y,color="g")
                plt.pause(0.01)

        plt.scatter(self.start.x,self.start.y,color="magenta")
        plt.scatter(self.goal.x,self.goal.y,color="blue")
        plt.pause(0.01)



        