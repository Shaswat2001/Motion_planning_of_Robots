import matplotlib.pyplot as plt
from Nodes import check_nodes
import os
import glob

class Visualize:
    '''
    For visualizations of the visited nodes and extracted path
    '''
    def __init__(self,start,goal,obs_map):
        
        self.start = start
        self.goal = goal
        self.obs_map = obs_map

        self.fig = plt.figure()
    
    def animate(self,algorithm,explNodes,path):
        self.plot_canvas(algorithm)
        self.explored_points(explNodes)
        self.shortest_path(path)
        plt.show()

    def animate_bi(self,algorithm,explNodes_frd,explNodes_back,path):
        self.plot_canvas(algorithm)
        self.explored_point_bi(explNodes_frd,explNodes_back)
        self.shortest_path(path)
        plt.show()

    def animate_rtaa_star(self,algorithm,visited_nodes,path):
        self.plot_canvas(algorithm)
        cl = self.color()
        path_total = []
        
        for k in range(len(path)):
            self.plot_visited(visited_nodes[k],cl[k])
            path_total+=path[k]
            self.shortest_path(path[k])
            plt.pause(0.5)

        self.shortest_path(path_total)
        plt.show()

    def animate_ara_star(self,algorithm,explNodes,path):
        self.plot_canvas(algorithm)
        cl_v, cl_p = self.color_list()
        
        for k in range(len(path)):
            self.explored_points(explNodes[k], cl_v[k])
            self.shortest_path(path[k], cl_p[k])
            plt.pause(0.5)

        plt.show()

    def animate_path(self,path):
        self.plot_canvas()
        self.shortest_path(path)
        plt.show()

    def plot_visited(self,visited,c = "lightgrey"):

        for nodes in visited:
            plt.plot(nodes.x,nodes.y,marker="o",color=c)
            plt.pause(0.005)

    def plot_canvas(self,algorithm):

        obsX = [obs.x for obs in self.obs_map]
        obsY = [obs.y for obs in self.obs_map]

        plt.scatter(obsX,obsY,marker='s',color = 'black')
        plt.scatter(self.start.x,self.start.y,color="green")
        plt.scatter(self.goal.x,self.goal.y,color="blue")
        plt.title(algorithm)
        plt.axis("equal")

    def explored_points(self,explNodes,clr = "grey"):
        
        for i in explNodes:
            if check_nodes(i,self.start):
                explNodes.remove(i)

        for i in explNodes:
            plt.plot(i.x,i.y,color=clr,marker='o')
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

            plt.plot(Node_bck.x,Node_bck.y,color="cyan",marker='o')
            plt.pause(0.01)

        while i!=len(explNodes_back)-1:
            node = explNodes_back[i]
            plt.plot(node.x,node.y,color="cyan",marker='o')
            plt.pause(0.01)
            i+=1

        while i!=len(explNodes_frd)-1:
            node = explNodes_frd[i]
            plt.plot(node.x,node.y,color="grey",marker='o')
            plt.pause(0.01)
            i+=1

    def shortest_path(self,path,clr="r"):

        path_x = [node.x for node in path]
        path_y = [node.y for node in path]
        plt.plot(path_x, path_y, linewidth='2', color=clr)

        plt.scatter(self.start.x,self.start.y,color="green")
        plt.scatter(self.goal.x,self.goal.y,color="blue")
        plt.pause(0.01)

    @staticmethod
    def color_list():
        cl_node = ['silver',
                'wheat',
                'lightskyblue',
                'royalblue',
                'slategray']
        cl_path = ['gray',
                'orange',
                'deepskyblue',
                'red',
                'm']
        return cl_node, cl_path
    
    @staticmethod
    def color():

        cl_v = ['silver',
                'wheat',
                'lightskyblue',
                'royalblue',
                'slategray']
        
        return cl_v
