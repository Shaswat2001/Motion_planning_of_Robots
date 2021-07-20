from graph import graph_conv,same_node_graph
from Nodes import Node,start,goal,check_nodes,check_NodeIn_list
from data_structure import PriorityQueue
from map import load_map
from Visualize import backtrack_list,add_path_Canvas,generate_video
import math
import cv2

def BFS_algorithm(graph,start,goal,maze_canvas):
    '''
    This function implements Breadth First Search Algorithm

    Arguments:
    start-- starting node (Instance of class Node)
    goal-- goal node (Instance of class Node)
    cost_graph-- Free configuration Space (Instance of class Graph)
    maze_canvas-- array representing the entire grid

    Returns:
    CLOSED-- List of nodes visited by the Algorithm
    backtrack_node-- Dict used to create the shortest path
    maze_canvas-- updated array 
    '''

    OPEN=PriorityQueue()
    # list of nodes visited by the algorithm
    CLOSED=[]
    video_count=0

    # vertices in the graph
    vertices=graph.get_vertices()
    #returns an instance of start Node from the graph
    start_vertex=same_node_graph(start,graph.graph)
    # Past Cost -- 0 for start Node
    past_cost={}
    past_cost[start_vertex]=0

    backtrack_node={}
    # Start Node with past cost is inserted into the queue
    OPEN.insert_pq(0, start_vertex)
    goal_reached=0
    # while the goal node is not reached
    while goal_reached!=1:
        # Node with lowest past_cost is removed from the queue
        current_ct,current_vt=OPEN.pop_pq()
        # the Node is added to the CLOSED list
        CLOSED.append(current_vt)
        # if the goal node is reached
        if check_nodes(current_vt,goal):
            goal_reached=1
            print("The goal node is found")
            return CLOSED,backtrack_node
        # the neighbours of current_vt from cost_graph
        neighbour=graph.get_neighbours(current_vt)
        
        for nbr in neighbour:
            # If the neighbour is not already visited
            if not check_NodeIn_list(nbr,CLOSED):

                # getting the same Node instance as used in cost_graph
                vertex_same=same_node_graph(current_vt,graph.graph)
                nbr_same=same_node_graph(nbr,graph.graph)

                # The node is added to the canvas
                cv2.circle(maze_canvas,nbr_same.get_inv_coordinates(),2,[255,0,0])
                # the Canvas is flipped to get the correct orientation
                flipVertical = cv2.rotate(maze_canvas, cv2.ROTATE_90_COUNTERCLOCKWISE)
                # the canvas is displayed
                cv2.imshow("MAP",flipVertical)
                # the canvas is saved as an image
                cv2.imwrite(f'BFS_Image/Image_{video_count}.jpg',flipVertical)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                video_count+=1

                # the neigbour node along with its parent and cost is added to the Dict
                backtrack_node[nbr_same]={}
                #past cost of neigbour is sum of past cost of parent and 1
                past_cost[nbr_same]=past_cost[vertex_same]+1
                backtrack_node[nbr_same][vertex_same]=past_cost[nbr_same]
                # Node along with the cost is added to the queue
                OPEN.insert_pq(past_cost[nbr_same], nbr_same)

                if check_nodes(nbr_same,goal):
                    goal_reached=1
                    print("The goal node is found")
                    CLOSED.append(nbr_same)
                    return CLOSED,backtrack_node

    # If a path Doesn't exit
    print("The Goal coudnt be reached")

def doBFS():
    path='BFS_Image/'
    # Make sure global variables are used
    global start,goal
    # Loads the canvas
    maze_canvas=load_map()
    CLOSED,backtrack_node=BFS_algorithm(graph_conv,start,goal,maze_canvas)
    # gets the list of nodes in the shortest path
    bkt_list=backtrack_list(backtrack_node,start,goal)
    maze_canvas=add_path_Canvas(bkt_list,maze_canvas,path)
    # Generates a video
    generate_video(path)

if __name__=="__main__":
    doBFS()
