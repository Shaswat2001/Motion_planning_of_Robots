from Nodes import Node
from map import Map
from graph import Graph
from MultiQuery import RRT,RRTConnect,ExtendRRT,RRTStar,DynamicRRT,FMTStar,RRTSharp,RRTStarSmart,InformedRRTStar
from SingleQuery import PRM,LazyPRM
from Visualize import Visualize

# Creating main window
if __name__ == "__main__":

    print("The Motion Planning Algorithm Library")
    planner = input("Enter the planning algorithm to run : ")
    grid_size=(51,31)
    delta = 0.5

    start_node=list(map(int,input("Enter the start node (x y)").split()))
    start=Node(*(x for x in start_node))
    goal_node=list(map(int,input("Enter the goal node (x y)").split()))
    goal=Node(*(x for x in goal_node))

    grid = Graph(grid_size,delta)

    if planner == "RRT":
        algorithm = RRT.RRT(start,goal,grid,10000,0.5)

    elif planner == "RRTConnect":
        algorithm = RRTConnect.RRTConnect(start,goal,grid,5000,0.5)

    elif planner == "DynamicRRT":
        algorithm = DynamicRRT.DynamicRRT(start,goal,grid,5000,0.5)

    elif planner == "ExtendRRT":
        algorithm = ExtendRRT.ExtendRRT(start,goal,grid,10000,0.5)
    
    elif planner == "RRTStar":
        algorithm = RRTStar.RRTStar(start,goal,grid,10000,0.5,1,10,20)

    elif planner == "RRTStarSmart":
        algorithm = RRTStarSmart.RRTStarSmart(start,goal,grid,10000,0.5,1,10,20)

    elif planner == "RRTSharp":
        algorithm = RRTSharp.RRTSharp(start,goal,grid,10000,1,0.5,10,20)

    elif planner == "FMTStar":
        algorithm = FMTStar.FMTStar(start,goal,grid,1,40,2500)

    elif planner == "PRM":
        algorithm = PRM.PRM(start,goal,grid,200,5,10)

    elif planner == "LazyPRM":
        algorithm = LazyPRM.LazyPRM(start,goal,grid,100,5,10)

    elif planner == "InformedRRTStar":
        algorithm = InformedRRTStar.InformedRRTStar(start,goal,grid,3000,1,0.5,1,12)
    
    algorithm.main()
