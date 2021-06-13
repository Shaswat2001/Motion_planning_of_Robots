from Nodes import Obstacles,grid_size,Node,calculate_distance,check_nodes,check_NodeIn_list
import numpy as np
from map import maze_canvas,check_obstacleNode_canvas

def same_node_graph(node,graph):

    for nodes in list(graph.keys()):
        if check_nodes(nodes,node):
            return nodes
    return 0


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
                node_same=same_node_graph(node,self.graph)
                return self.graph[node_same]

def neighbour_node(point):
    global grid_size

    (max_x,max_y)=grid_size
    (x,y)=point.get_coordinates()
    graph={}

    # For origin (0,0)
    if x==0 and y==0:
        graph[point]={(x+1,y),(x,y+1)}
    # For last coordinate in the grid
    elif x==max_x and y==max_y:
        graph[point]={(x-1,y),(x,y-1)}
    # For points in the x=0 and 0< y <max_y
    elif x==0 and y!=0 and y!=max_y:
        graph[point]={(x+1,y),(x,y-1),(x,y+1)}
    # For points in the y=0 and 0< x <max_x
    elif y==0 and x!=0 and x!=max_x:
        graph[point]={(x-1,y),(x+1,y),(x,y+1)}
    # For point (0,max_y)
    elif x==0 and y==max_y:
        graph[point]={(x,y-1),(x+1,y)}
    # For point (max_x,0)
    elif y==0 and x==max_x:
        graph[point]={(x-1,y),(x,y+1)}
    # For points in the y=max_y and 0< x <max_x
    elif y==max_y and x!=0 and x!=max_x:
        graph[point]={(x,y-1),(x+1,y),(x-1,y)}
    # For points in the x=max_x and 0< y <max_y
    elif x==max_x and y!=0 and y!=max_y:
        graph[point]={(x-1,y),(x,y+1),(x,y-1)}
    # For rest of the case
    else:
        graph[point]={(x+1,y),(x-1,y),(x,y+1),(x,y-1)}

    return graph

def check_edge_CollisionFree(parent,neighbour):
    parent=parent.get_coordinates()
    nbr=neighbour.get_coordinates()
    collision=False
    ot=[]
    min_x=min(parent[0],nbr[0])
    max_x=max(parent[0],nbr[0])
    min_y=min(parent[1],nbr[1])
    max_y=max(parent[1],nbr[1])
    if parent[0]!=nbr[0]:
        slope=(parent[1]-nbr[1])/(parent[0]-nbr[0])
        for x in [min_x+(max_x-min_x)*(i/29) for i in range(30)]:
            ot.append(nbr[1]+slope*(x-nbr[0]))
    else:
        for j in [min_y+(max_y-min_y)*(i/29) for i in range(30)]:
            ot.append(j)

    for x in [min_x+(max_x-min_x)*(i/29) for i in range(30)]:
        for y in ot:
            if(130+x>=y) and (290-7*x<=y) and ((17/3)*x-90<=y):
                collision=True

            if (x>=90 and 5*x-360<=y and y<=155) or (x>=90 and(x+530>=4*y) and ((5/6)*x+(170/3)<=y) and x<=130):
                collision=True

            if x>=120 and x<=160 and y>=35 and y<=130:
                if (x-10)>=y:
                    if x-400<=-2*y:
                        if 3*x-360<=y:
                            if x-60<=y or (-7/3)*x+(1120/3)>=y:
                                if (-2/5)*x +93<=y:
                                    collision=True

            if (2*x-340>=y) and ((-5/2)*x+605>=y) and (x-350>=-4*y):
                collision=True

            if (-3*x+960>=y) and ((2/11)*x+(1460/11)>=y) and ((7/2)*x-(565)>=y) and (x+580<=5*y):
                collision=True

            if collision==True:
                break

    return collision

def generate_cost_graph(maze_canvas):

    cost_graph={}

    for i in range(grid_size[0]+1):
        for j in range(grid_size[1]+1):

            node=Node(i,j)
            if not check_obstacleNode_canvas(node,maze_canvas):

                cost_graph[node]={}
                neighbour=list(neighbour_node(node).values())

                for nbr in neighbour[0]:
                    nbr_node=Node(nbr[0],nbr[1])
                    if not check_obstacleNode_canvas(nbr_node,maze_canvas):
                        dist=calculate_distance(node,nbr_node)
                        cost_graph[node][nbr_node]=dist

    cost_graph_conv=Graph(cost_graph)

    return cost_graph_conv

def generate_graph(maze_canvas):

    graph={}

    for i in range(grid_size[0]+1):
        for j in range(grid_size[1]+1):

            node=Node(i,j)
            if not check_obstacleNode_canvas(node,maze_canvas):

                graph[node]=[]
                neighbour=list(neighbour_node(node).values())

                for nbr in neighbour[0]:
                    nbr_node=Node(nbr[0],nbr[1])
                    if not check_obstacleNode_canvas(nbr_node,maze_canvas):
                        graph[node].append(nbr_node)

    graph_conv=Graph(graph)

    return graph_conv

graph_conv=generate_graph(maze_canvas)

cost_graph_conv=generate_cost_graph(maze_canvas)
