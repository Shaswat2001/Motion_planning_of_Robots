import math

class Node:

    def __init__(self,x,y):
        self.x=x
        self.y=y

        self.cost=0
        self.parent=None

    def get_coordinates(self):
        return (self.x,self.y)

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

def check_NodeIn_list(node,check_list):

    for nodes in check_list:
        if check_nodes(nodes,node):
            return True
    return False

grid_size=(300,200)

start_node=list(map(int,input("Enter the start node (x y)").split()))
start=Node(*(x for x in start_node))
goal_node=list(map(int,input("Enter the goal node (x y)").split()))
goal=Node(*(x for x in goal_node))

Obstacles=True
