import math 

def euclidean_heuristic(node1,node2):
    '''
    This Function calculates the Euclidean distance between two nodes

    Arguments:
    node1-- Instance of class Node
    node2-- Instance of class Node

    Returns:
    euc_dist-- Euclidean distance between node1 and node2
    '''
    # Coordinates in Node1
    (x1,y1)=node1.get_coordinates()
    #Coordinates in Node2
    (x2,y2)=node2.get_coordinates()
    #The Euclidean distance rounded upto 2 decimal places
    euc_dist=round(math.sqrt((x1-x2)**2+(y1-y2)**2),2)

    return euc_dist

def manhattan_heuristic(node1,node2):
    '''
    This Function calculates the Manhattan distance between two nodes

    Arguments:
    node1-- Instance of class Node
    node2-- Instance of class Node

    Returns:
    man_dist-- Manhattan distance between node1 and node2
    '''
    # Coordinates in Node1
    (x1,y1)=node1.get_coordinates()
    #Coordinates in Node2
    (x2,y2)=node2.get_coordinates()
    #The Manhattan distance
    man_dist=abs(x1-x2)+abs(y1-y2)

    return man_dist

def diagonal_heuristic(node1,node2,D=1,D2=math.sqrt(2)):
    '''
    This Function calculates the Diagonal distance between two nodes

    Arguments:
    node1-- Instance of class Node
    node2-- Instance of class Node

    Returns:
    diag_dist-- Diagonal distance between node1 and node2
    '''
    # Coordinates in Node1
    (x1,y1)=node1.get_coordinates()
    #Coordinates in Node2
    (x2,y2)=node2.get_coordinates()

    dx=abs(x1-x2)
    dy=abs(y1-y2)
    #The Diagonal distance
    diag_dist=D*(dx+dy)+(D2-2*D)*min(dx,dy)

    return diag_dist