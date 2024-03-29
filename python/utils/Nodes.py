import math

class Node:
    '''
    The class describes a cell of the grid
    '''
    def __init__(self,x,y,theta=None):
        self.x=x
        self.y=y
        self.theta = theta
        self.cost = 0
        self.parent = None

    def get_coordinates(self):
        '''
        Returns the coordinates of a node
        '''
        return (self.x,self.y,self.theta)

    def get_inv_coordinates(self):
        '''
        Returns the inverted coordinates of a node
        '''
        return (self.y,self.x,self.theta)
    

def calculate_distance(node1,node2):
    '''
    Returns the euclidean distance between two nodes
    '''
    # the x and y coordinates of a node
    (x1,y1,_)=node1.get_coordinates()
    (x2,y2,_)=node2.get_coordinates()

    # euclidean distance
    euc_distance=round(math.sqrt((x1-x2)**2+(y1-y2)**2),2)

    return euc_distance

def check_nodes(node1,node2):
    '''
    Checks if the nodes are equal
    '''
    # checks if both x and y coordinates are the same
    if node1.x==node2.x and node1.y==node2.y:
        return True

    return False

def check_NodeIn_list(node,check_list):
    '''
    Check if the nodes exist in a list
    '''
    # loops through the list
    for nodes in check_list:
        # checks if two nodes are equal
        if check_nodes(nodes,node):
            return True
    return False

def getSameNode(node,node_list):

    for nodes in node_list:

        if check_nodes(nodes,node):
            return nodes
    
    return None
