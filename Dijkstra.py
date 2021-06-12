from graph import cost_graph_conv,same_node_graph
from Nodes import Node,start,goal,check_nodes,check_NodeIn_list
from data_structure import PriorityQueue
from map import maze_canvas
import cv2
import math

def Dijkstra_search(cost_graph,start,goal):

    OPEN=PriorityQueue()
    CLOSED=[]
    backtrack_node={}
    vertices=cost_graph.get_vertices()
    past_cost={nodes:math.inf for nodes in vertices}
    start_vertex=same_node_graph(start,cost_graph.graph)
    past_cost[start_vertex]=0
    OPEN.insert_pq(0, start_vertex)

    while OPEN.len_pq()>0:
        current_ct,current_vt=OPEN.pop_pq()
        CLOSED.append(current_vt)

        if check_nodes(current_vt,goal):
            print("The goal node is found")
            return CLOSED,backtrack_node

        neighbour=cost_graph.get_neighbours(current_vt)
        for nbr,cost in neighbour.items():

            if not check_NodeIn_list(nbr,CLOSED):

                vertex_same=same_node_graph(current_vt,cost_graph.graph)
                tentatative_distance=past_cost[vertex_same]+cost
                nbr_same=same_node_graph(nbr,cost_graph.graph)
                if past_cost[nbr_same]>tentatative_distance:
                    cv2.circle(maze_canvas,nbr_same.get_coordinates(),1,[125,125,125])
                    flipVertical = cv2.rotate(maze_canvas, cv2.ROTATE_90_COUNTERCLOCKWISE)
                    cv2.imshow("MAP",flipVertical)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    backtrack_node[nbr_same]={}
                    past_cost[nbr_same]=tentatative_distance
                    backtrack_node[nbr_same][vertex_same]=tentatative_distance
                    OPEN.insert_pq(tentatative_distance, nbr_same)
                    if check_nodes(nbr_same,goal):
                        print("The goal node is found")
                        return CLOSED,backtrack_node

    print("The Goal coudnt be reached")

def doDijkstra():
    global start,goal
    CLOSED,backtrack_node=Dijkstra_search(cost_graph_conv,start,goal)
    print(len(backtrack_node))


if __name__=="__main__":
    doDijkstra()
