from Nodes import Node
from map import Map
from graph import Graph
import RRT
from Visualize import Visualize

# Creating main window
if __name__ == "__main__":

    print("The Motion Planning Algorithm Library")
    planner = "RRT"
    grid_size=(300,200)

    start_node=list(map(int,input("Enter the start node (x y)").split()))
    start=Node(*(x for x in start_node))
    goal_node=list(map(int,input("Enter the goal node (x y)").split()))
    goal=Node(*(x for x in goal_node))

    grid = Graph(grid_size)

    if planner == "RRT":
        algorithm = RRT.RRT(start,goal,grid,10000,2)
    
    algorithm.main()