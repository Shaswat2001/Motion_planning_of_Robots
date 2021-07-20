from graph import cost_graph_conv,same_node_graph
from Nodes import Node,start,goal,check_nodes,check_NodeIn_list
from data_structure import PriorityQueue
from map import load_map
from Visualize import backtrack_list,add_path_Canvas,generate_video
import cv2
import math

def Dijkstra_search(cost_graph,start,goal,maze_canvas):
    '''
    This function implements Dijkstra Search Algorithm

    Arguments:
    start-- starting node (Object of class Node)
    goal-- goal node (Object of class Node)
    cost_graph-- Free configuration Space (Instance of class Graph)
    maze_canvas-- array representing the entire grid

    Returns:
    CLOSED-- List of nodes visited by the Algorithm
    backtrack_node-- Dict used to create the shortest path
    maze_canvas-- a numpy array
    '''
    OPEN=PriorityQueue()
    # list of nodes visited by the algorithm
    CLOSED=[]
    video_count=0
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
                    cv2.circle(maze_canvas,nbr_same.get_inv_coordinates(),2,[255,0,0])
                    # the Canvas is flipped to get the correct orientation
                    flipVertical = cv2.rotate(maze_canvas, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    # the canvas is displayed
                    cv2.imshow("MAP",flipVertical)
                    # the canvas is saved as an image
                    cv2.imwrite(f'Dijkstra_image/Image_{video_count}.jpg',flipVertical)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                    video_count+=1
                    # the neigbour node along with its parent and cost is added to the Dict
                    backtrack_node[nbr_same]={}
                    past_cost[nbr_same]=tentatative_distance
                    backtrack_node[nbr_same][vertex_same]=tentatative_distance
                    # Node along with the cost is added to the queue
                    OPEN.insert_pq(tentatative_distance, nbr_same)
                    #if the goal node is reached
                    if check_nodes(nbr_same,goal):
                        print("The goal node is found")
                        return CLOSED,backtrack_node

    # If a path Doesn't exit
    print("The Goal coudnt be reached")

def doDijkstra():

    path='Dijkstra_image/'
    # Make sure global variables are used
    global start,goal
    # Loads the canvas
    maze_canvas=load_map()
    CLOSED,backtrack_node=Dijkstra_search(cost_graph_conv,start,goal,maze_canvas)
    # gets the list of nodes in the shortest path
    bkt_list=backtrack_list(backtrack_node,start,goal)
    maze_canvas=add_path_Canvas(bkt_list,maze_canvas,path)
    # Generates a video
    generate_video(path)


if __name__=="__main__":
    doDijkstra()
