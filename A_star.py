from graph import cost_graph_conv,same_node_graph
from Nodes import Node,start,goal,check_nodes,check_NodeIn_list
from data_structure import PriorityQueue
from map import maze_canvas
from Visualize import plot_tree,backtrack_list,add_path_Canvas,generate_video
import math
import cv2

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

def A_star_search(cost_graph,start,goal):
    '''
    This function implements A* Search Algorithm

    Arguments:
    start-- starting node (Instance of class Node)
    goal-- goal node (Instance of class Node)
    cost_graph-- Free configuration Space (Instance of class Graph)

    Returns:
    CLOSED-- List of nodes visited by the Algorithm
    backtrack_node-- Dict used to create the shortest path
    '''
    video_count=0
    global maze_canvas
    OPEN=PriorityQueue()
    # list of nodes visited by the algorithm
    CLOSED=[]
    backtrack_node={}

    # vertices in the graph
    vertices=cost_graph.get_vertices()
    #returns an instance of start Node from the graph
    start_vertex=same_node_graph(start,cost_graph.graph)

    # Past Cost -- 0 for start Node: infinity for rest
    past_cost={nodes:math.inf for nodes in vertices}
    past_cost[start_vertex]=0
    # Start Node with past cost is inserted into the queue
    OPEN.insert_pq(0, start_vertex)

    while OPEN.len_pq()>0:
        # Node with lowest past_cost is removed from the queue
        current_ct,current_vt=OPEN.pop_pq()
        # the Node is added to the CLOSED list
        CLOSED.append(current_vt)
        # if the goal node is reached
        if check_nodes(current_vt,goal):
            print("The goal node is found")
            return CLOSED,backtrack_node

        # the neighbours of current_vt from cost_graph
        neighbour=cost_graph.get_neighbours(current_vt)
        for nbr,cost in neighbour.items():
            # If the neighbour is not already visited
            if not check_NodeIn_list(nbr,CLOSED):

                # getting the same Node instance as used in cost_graph
                vertex_same=same_node_graph(current_vt,cost_graph.graph)
                nbr_same=same_node_graph(nbr,cost_graph.graph)

                # the tentatative_distance is calculated
                tentatative_distance=past_cost[vertex_same]+cost
                # If the past_cost is greater then the tentatative_distance
                if past_cost[nbr_same]>tentatative_distance:
                    # The node is added to the canvas
                    cv2.circle(maze_canvas,nbr_same.get_coordinates(),2,[255,0,0])
                    # the Canvas is flipped to get the correct orientation
                    flipVertical = cv2.rotate(maze_canvas, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    # the canvas is displayed
                    cv2.imshow("MAP",flipVertical)
                    # the canvas is saved as an image
                    cv2.imwrite(f'A_star_image/Image_{video_count}.jpg',flipVertical)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                    video_count+=1
                    # the neigbour node along with its parent and cost is added to the Dict
                    backtrack_node[nbr_same]={}
                    past_cost[nbr_same]=tentatative_distance
                    backtrack_node[nbr_same][vertex_same]=tentatative_distance
                    # Chosen heuristic is added to the tentatative_distance before adding it to the queue
                    tentatative_distance+=manhattan_heuristic(goal,nbr_same)
                    # Node along with the cost is added to the queue
                    OPEN.insert_pq(tentatative_distance, nbr_same)
                    #if the goal node is reached
                    if check_nodes(nbr_same,goal):
                        CLOSED.append(nbr_same)
                        print("The goal node is found")
                        return CLOSED,backtrack_node
    # If a path Doesn't exit 
    print("The Goal coudnt be reached")

def doA_star():
    # Make sure global variables are used
    global start,goal,maze_canvas
    CLOSED,backtrack_node=A_star_search(cost_graph_conv,start,goal)
    # gets the list of nodes in the shortest path
    bkt_list=backtrack_list(backtrack_node,start,goal)
    maze_canvas=add_path_Canvas(bkt_list,maze_canvas)
    path='A_star_image/'
    # Generates a video
    generate_video(path)

if __name__=="__main__":
    doA_star()
