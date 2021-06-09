from Nodes import Obstacles,grid_size,check_node_obstacle_list,Node,calculate_distance
import numpy as np
from map import maze_canvas

def check_obstacle_canvas(node,canvas):

    if (canvas[node.x][node.y]==[0,0,0]).all():
        return True
    else:
        return False

class Graph:

    def __init__(self,graph_dict):
        self.graph=graph_dict

    def get_vertices(self):
        vertices=list(self.graph.keys())
        return vertices

    def get_neighbours(self,node):
        vertices=list(self.graph.keys())
        for nodes in vertices:
            if check_nodes(nodes,node):
                return self.graph[node]

def neighbour_node(node):
    x=node.x
    y=node.y
    graph={}

    # For origin (0,0)
    if x==0 and y==0:
        graph[point]={(x+1,y),(x,y+1),(x+1,y+1)}
    # For last coordinate in the grid
    elif x==max_x and y==max_y:
        graph[point]={(x-1,y),(x-1,y-1),(x,y-1)}
    # For points in the x=0 and 0< y <max_y
    elif x==0 and y!=0 and y!=max_y:
        graph[point]={(x+1,y),(x,y-1),(x,y+1)}
    # For points in the y=0 and 0< x <max_x
    elif y==0 and x!=0 and x!=max_x:
        graph[point]={(x-1,y),(x+1,y),(x,y+1)}
    # For point (0,max_y)
    elif x==0 and y==max_y:
        graph[point]={(x,y-1),(x+1,y),(x+1,y-1)}
    # For point (max_x,0)
    elif y==0 and x==max_x:
        graph[point]={(x-1,y),(x,y+1),(x-1,y+1)}
    # For points in the y=max_y and 0< x <max_x
    elif y==max_y and x!=0 and x!=max_x:
        graph[point]={(x,y-1),(x+1,y),(x-1,y)}
    # For points in the x=max_x and 0< y <max_y
    elif x==max_x and y!=0 and y!=max_y:
        graph[point]={(x-1,y),(x,y+1),(x,y-1)}
    # For rest of the case
    else:
        graph[point]={(x+1,y),(x-1,y),(x,y+1),(x,y-1),(x+1,y+1),(x-1,y+1),(x-1,y-1),(x+1,y-1)}

    return graph

def generate_cost_graph(maze_canvas):

    cost_graph={}

    for i in range(grid_size[0]):
        for j in range(grid_size[1]):

            node=Nodes(i,j)
            if not check_obstacle_canvas(node,maze_canvas):

                cost_graph[node]={}
                neighbour=list(neighbour_node(node).values())

                for nbr in neighbour.values():
                    nbr_node=Node(nbr[0],nbr[1])
                    if not check_obstacle_canvas(nbr_node,maze_canvas):
                        dist=calculate_distance(node,nbr_node)
                        cost_graph[node][nbr_node]=dist

    return cost_graph
