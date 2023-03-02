import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.spatial.transform import Rotation as Rot
from Nodes import check_nodes
import math
import numpy as np
import os
import glob

class Visualize:

    def __init__(self,start,goal,obs_boundary,obs_rectangle,obs_circle):
        
        self.start = start
        self.goal = goal
        self.obs_bound = obs_boundary
        self.obs_rectangle = obs_rectangle
        self.obs_circle = obs_circle

        self.fig, self.ax = plt.subplots()
    
    def animate(self,algorithm,visited,path):
        self.plot_canvas(algorithm)
        self.plot_visited(visited)
        self.shortest_path(path)
        plt.show()

    def animate_connect(self,algorithm,treeA,treeB,path):
        self.plot_canvas(algorithm)
        self.draw_tree_connect(treeA,treeB)
        self.shortest_path(path)
        plt.show()

    def animate_rrt_star(self,algorithm,visited,path):
        self.plot_canvas(algorithm)
        self.plot_visited(visited)
        self.shortest_path(path)
        plt.show()

    def animate_prm(self,algorithm,grid,nodes,path):
        self.plot_canvas(algorithm)
        self.plot_random_nodes(nodes)
        self.draw_prm_grid(grid)
        self.shortest_path(path)
        plt.show()

    def animate_fmt_star(self,algorithm,node_list,visited,path):
        self.plot_canvas(algorithm)
        self.plot_random_nodes(node_list)
        self.plot_visited(visited)
        self.shortest_path(path)
        plt.show()
    
    def animate_irrt_star(self,algorithm,node_list,visited,path):
        self.plot_canvas(algorithm)
        self.plot_random_nodes(node_list)
        self.plot_visited(visited)
        self.shortest_path(path)
        plt.show()

    def plot_canvas(self,algorithm):

        for (ox, oy, w, h) in self.obs_bound:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        for (ox, oy, r) in self.obs_circle:
            self.ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
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

        path_x = [node.x for node in path]
        path_y = [node.y for node in path]
        plt.plot(path_x, path_y, linewidth='2', color="r")

        plt.scatter(self.start.x,self.start.y,color="magenta")
        plt.scatter(self.goal.x,self.goal.y,color="blue")
        plt.pause(0.01)

    def plot_visited(self,visited):

        for nodes in visited:

            if nodes.parent:
                root=nodes.get_coordinates()
                nbr=nodes.parent.get_coordinates()
                plt.plot([root[0],nbr[0]],[root[1],nbr[1]],'-g')
                # plt.pause(0.00001)

    @staticmethod
    def draw_ellipse(x_center, c_best, dist, theta):
        a = math.sqrt(c_best ** 2 - dist ** 2) / 2.0
        b = c_best / 2.0
        angle = math.pi / 2.0 - theta
        cx = x_center[0]
        cy = x_center[1]
        t = np.arange(0, 2 * math.pi + 0.1, 0.1)
        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]
        rot = Rot.from_euler('z', -angle).as_dcm()[0:2, 0:2]
        fx = rot @ np.array([x, y])
        px = np.array(fx[0, :] + cx).flatten()
        py = np.array(fx[1, :] + cy).flatten()
        plt.plot(cx, cy, ".b")
        plt.plot(px, py, linestyle='--', color='darkorange', linewidth=2) 