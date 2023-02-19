import matplotlib.pyplot as plt
import matplotlib.patches as patches
from Nodes import check_nodes
import os
import glob

class Visualize:

    def __init__(self,start,goal,obs_boundary,obs_rectangle,obs_circle):
        
        self.start = start
        self.goal = goal
        self.obs_bound = obs_boundary
        self.obs_rectangle = obs_rectangle
        self.obs_circle = obs_circle

        self.fig = plt.figure()
    
    def animate(self,explNodes,path):
        self.plot_canvas()
        self.explored_points(explNodes)
        self.shortest_path(path)
        plt.show()

    def animate_path(self,path):
        self.plot_canvas()
        self.shortest_path(path)
        plt.show()

    def plot_canvas(self):

        fig, ax = plt.subplots()

        for (ox, oy, w, h) in self.obs_bound:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.obs_rectangle:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        for (ox, oy, r) in self.obs_circle:
            ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        plt.scatter(self.start.x,self.start.y,color="green")
        plt.scatter(self.goal.x,self.goal.y,color="blue")
        plt.axis("equal")

    def explored_points(self,explNodes):
        
        for i in explNodes:
            if check_nodes(i,self.start):
                explNodes.remove(i)

        for i in explNodes:
            plt.plot(i.x,i.y,color="grey",marker='o')
            plt.pause(0.01)

    def draw_tree(self,tree):

        # Loop through the vertices
        for prt,node in tree:
            # Coordinate of 'i' node
            root=prt.get_coordinates()
            nbr=node.get_coordinates()
            plt.plot([root[0],nbr[0]],[root[1],nbr[1]],linewidth='1', color="pink")
            plt.pause(0.01)

    def draw_graph(self,graph):

        vertices=graph.get_vertices()
        # Loop through the vertices
        for i in vertices:
            # Coordinate of 'i' node
            root=i.get_coordinates()
            # Loop through the neighbours of 'i' node
            for j in graph.get_neighbours(i):

                nbr=j.get_coordinates()

                plt.plot([root[0],nbr[0]],[root[1],nbr[1]],linewidth='1', color="pink")
        
        plt.pause(0.01)

    def shortest_path(self,path):

        path_x = [node.x for node in path]
        path_y = [node.y for node in path]
        plt.plot(path_x, path_y, linewidth='2', color="r")

        plt.scatter(self.start.x,self.start.y,color="green")
        plt.scatter(self.goal.x,self.goal.y,color="blue")
        plt.pause(0.01)
