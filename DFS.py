from graph import graph_conv,same_node_graph
from Nodes import Node,start,goal,check_nodes,check_NodeIn_list
import math
import sys

class DFS_algorithm:

    def __init__(self,graph,goal):

        self.graph=graph
        self.vertices=graph.get_vertices()
        self.visited=[]
        self.goal_reached=False
        self.goal_Node=goal

    def SolveDFS(self,node):

        if check_nodes(node,self.goal_Node):
            self.goal_reached=True
            print("Goal Reached")
            return self.visited

        if len(self.visited)==len(self.vertices):
            print("Goal Not Reached")
        else:
            node_vertex=same_node_graph(node,self.graph.graph)
            self.visited.append(node_vertex)
            neighbours=self.graph.get_neighbours(node_vertex)
            for i in self.visited:
                print(i.get_coordinates())
            if neighbours is not None:
                for nbr in neighbours:
                    if not check_NodeIn_list(nbr,self.visited):
                        if not self.goal_reached:
                            self.SolveDFS(nbr)

def doDFS_algorithm():
    global goal,start
    DFS=DFS_algorithm(graph_conv,goal)
    visited_nodes=DFS.SolveDFS(start)
    for i in visited:
        print(i.get_coordinates())

if __name__=="__main__":
    doDFS_algorithm()
