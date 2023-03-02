from Nodes import Node
from map import Map
from graph import Graph
from MultiQuery import RRT,RRTConnect,ExtendRRT,RRTStar,DynamicRRT,FMTStar,RRTSharp,RRTStarSmart,InformedRRTStar
from SingleQuery import PRM,LazyPRM
from Visualize import Visualize

# Creating main window
if __name__ == "__main__":

    print("The Motion Planning Algorithm Library")
    grid_size=(50,30)
    delta = 0.5

    grid = Graph(grid_size,delta)

    print(grid.check_node_CollisionFree(Node(20,13)))