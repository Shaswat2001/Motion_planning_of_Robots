import matplotlib.pyplot as plt
from Nodes import check_nodes
import os
import glob

class Visualize:

    def __init__(self,start,goal,obs_map):
        
        self.start = start
        self.goal = goal
        self.obs_map = obs_map

        self.fig = plt.figure()
    
    def animate(self,explNodes,path):
        self.plot_canvas()
        self.explored_points(explNodes)
        self.shortest_path(path)
        plt.show()

    def animate_bi(self,explNodes_frd,explNodes_back,path):
        self.plot_canvas()
        self.explored_point_bi(explNodes_frd,explNodes_back)
        self.shortest_path(path)
        plt.show()

    def animate_path(self,path):
        self.plot_canvas()
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
            plt.pause(0.01)
    
    def explored_point_bi(self,explNodes_frd,explNodes_back):
        
        for i in explNodes_frd:
            if check_nodes(i,self.start):
                explNodes_frd.remove(i)

        for i in explNodes_back:
            if check_nodes(i,self.goal):
                explNodes_back.remove(i)

        for i in range(min(len(explNodes_back),len(explNodes_frd))):

            Node_frd = explNodes_frd[i]
            Node_bck = explNodes_back[i]
            plt.plot(Node_frd.x,Node_frd.y,color="grey",marker='o')

            plt.plot(Node_bck.x,Node_bck.y,color="darkgrey",marker='o')
            plt.pause(0.01)

        while i!=len(explNodes_back)-1:
            node = explNodes_back[i]
            plt.plot(node.x,node.y,color="grey",marker='o')
            plt.pause(0.01)
            i+=1

        while i!=len(explNodes_frd)-1:
            node = explNodes_frd[i]
            plt.plot(node.x,node.y,color="grey",marker='o')
            plt.pause(0.01)
            i+=1

    def draw_tree(self,tree):

        # Loop through the vertices
        for prt,node in tree:
            # Coordinate of 'i' node
            root=prt.get_coordinates()
            nbr=node.get_coordinates()
            plt.plot([root[0],nbr[0]],[root[1],nbr[1]],linewidth='1', color="pink")
        plt.pause(0.01)
        plt.show()

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
