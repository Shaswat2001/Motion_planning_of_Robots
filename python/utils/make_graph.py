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
        self.plot_non_holonomic_visited(path,True)
        self.shortest_path(path,algorithm)
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

    def shortest_path(self,path,algorithm):

        int_node_x = []
        int_node_y = []
        for i in reversed(range(1,len(path))):
        
            theta = path[i]["vertex"].theta

            for x,y,yaw in path[i]["intermediate_nodes"]:

                plt.cla()
                self.plot_canvas(algorithm)
                self.plot_non_holonomic_visited(path)
                int_node_x.append(round(x,2))
                int_node_y.append(round(y,2))
                plt.plot(int_node_x, int_node_y,color="r")
                self.plot_robot(x,y,yaw,25)
                plt.pause(0.0001)

        plt.scatter(self.start.x,self.start.y,color="magenta")
        plt.scatter(self.goal.x,self.goal.y,color="blue")
        plt.pause(0.0001)

    def plot_non_holonomic_visited(self,path,pause = False):

        for i in reversed(range(1,len(path))):

            start = path[i]["vertex"]
            neighbours = self.drive.get_neighbours(start)
            # theta = np.rad2deg(math.atan2(end.y-start.y,end.x-start.x))%360
            for _,value in neighbours.items():

                int_nodes = value["intermediate_nodes"]

                int_node_x = []
                int_node_y = []
                for i in range(len(int_nodes)):
                    int_node_x.append(round(int_nodes[i][0],2))
                    int_node_y.append(round(int_nodes[i][1],2))
                plt.plot(int_node_x, int_node_y,color="g")
                if pause:
                    plt.pause(0.01)

        plt.scatter(self.start.x,self.start.y,color="magenta")
        plt.scatter(self.goal.x,self.goal.y,color="blue")
        if pause:
            plt.pause(0.01)

    def plot_curve(self,g_x,g_y,g_theta,mode):

        for (x,y,theta) in zip(g_x,g_y,g_theta):
            plt.cla()
            plt.plot(g_x*100, g_y*100, label="".join(mode),color="g")
            self.plot_robot(x*100,y*100,theta,13)
            plt.scatter(self.start[0]*100, self.start[1]*100,color="magenta")
            plt.scatter(self.goal[0]*100, self.goal[1]*100,color="blue")
            plt.legend()
            plt.grid(True)
            plt.axis("equal")
            plt.title("Dubins Curve")
            plt.pause(0.0001)
        plt.show()

    def plot_robot(self,x,y,theta,radius):

        car_color = '-k'
        c, s = math.cos(theta), math.sin(theta)

        cir_theta = np.linspace(0 , 2*np.pi , 150)

        outline_x = x + radius*np.cos(cir_theta)
        outline_y = y + radius*np.sin(cir_theta)

        arrow_x, arrow_y, arrow_theta = c*1.5 + x, s*1.5 + y, theta
        self.plot_arrow(arrow_x, arrow_y, arrow_theta)
        plt.plot(outline_x, outline_y, car_color)

    def plot_arrow(self,x,y,theta,length = 30.0,width=3.5, fc="r", ec="k"):

        plt.arrow(x, y, length*math.cos(theta), length*math.sin(theta),
                  fc=fc, ec=ec, head_width=width, head_length=width, alpha=0.8)




        