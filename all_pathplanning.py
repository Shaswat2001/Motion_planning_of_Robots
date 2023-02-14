from Nodes import Node
from map import Map
from graph import Graph
from HeuristicSearch import Astar,Dijkstra,PRM,BidirectionalAstar
from IncrementalSearch import Dstar,LPAstar
from SamplingBased import RRT
from Visualize import Visualize

# Creating main window
if __name__ == "__main__":

    print("The Motion Planning Algorithm Library")
    planner = "LPAstar"
    grid_size=(51,31)

    start_node=list(map(int,input("Enter the start node (x y)").split()))
    start=Node(*(x for x in start_node))
    goal_node=list(map(int,input("Enter the goal node (x y)").split()))
    goal=Node(*(x for x in goal_node))

    grid = Graph(grid_size)

    if planner == "PRM":
        gridPRM = PRM.PRM(grid,start,goal)
        grid = gridPRM.generate_PRM()
        print(grid.get_vertices())

        algorithm = Astar.Astar(start,goal,grid)

    elif planner == "Astar":
        algorithm = Astar.Astar(start,goal,grid)

    elif planner == "Dstar":
        algorithm = Dstar.Dstar(start,goal,grid)

    elif planner == "BiAstar":
        algorithm = BidirectionalAstar.BidirectionalAstar(start,goal,grid)

    elif planner == "Dijkstra":
        algorithm = Dijkstra.Dijkstra(start,goal,grid)

    elif planner == "RRT":
        algorithm = RRT.RRT(start,goal,grid,200,2)

    elif planner == "LPAstar":
        algorithm = LPAstar.LPAstar(start,goal,grid)
    
    algorithm.main()
