import math

grid_size=(300,200)

start=list(map(int,input("Enter the start node (x y)").split()))

goal=list(map(int,input("Enter the goal node (x y)").split()))

Obstacles=True

class Node:

    def __init__(self,x,y):
        self.x=x
        self.y=y

        self.cost=0
        self.parent=None

def calculate_distance(node1,node2):

    x1=node1.x
    y1=node1.y

    x2=node2.x
    y2=node2.y

    euc_distance=round(math.sqrt((x1-x2)**2+(y1-y2)**2),2)

    return euc_distance

def check_nodes(node1,node2):

    if node1.x==node2.x and node1.y==node2.y:
        return True

    return False

def check_node_obstacle_list(node,obs_list):

    for nodes in obs_list:
        if check_nodes(nodes,node):
            return True

    return False
