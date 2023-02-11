import matplotlib.pyplot as plt
from Nodes import check_nodes
import os
import glob

class Visualize:

    def __init__(self,start,goal,obs_map):
        
        self.start = start
        self.goal = goal
        self.obs_map = obs_map
    
    def animate(self,explNodes,path):
        self.plot_canvas()
        self.explored_points(explNodes)
        self.shortest_path(path)
        plt.show()

    def plot_canvas(self):

        obsX = [obs.x for obs in self.obs_map]
        obsY = [obs.y for obs in self.obs_map]

        plt.scatter(obsX,obsY,marker='s',color = 'black')
        plt.scatter(self.start.x,self.start.y,color="green")
        plt.scatter(self.goal.x,self.goal.y,color="blue")
        plt.axis("equal")

    def explored_points(self,explNodes):
        
        for i in explNodes:
            if check_nodes(i,self.start):
                explNodes.remove(i)

        for i in explNodes:
            plt.plot(i.x,i.y,color="grey",marker='o')
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])
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
        plt.plot(path_x, path_y, linewidth='3', color="r")

        plt.scatter(self.start.x,self.start.y,color="green")
        plt.scatter(self.goal.x,self.goal.y,color="blue")
        plt.pause(0.01)
