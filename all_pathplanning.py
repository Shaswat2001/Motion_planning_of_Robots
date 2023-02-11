from Nodes import Node
from map import Map
from graph import Graph,check_edge_CollisionFree
from HeuristicSearch import Astar,Dijkstra,PRM,BidirectionalAstar
from Visualize import Visualize

# Creating main window
if __name__ == "__main__":

    print("The Motion Planning Algorithm Library")
    planner = "BiAstar"
    grid_size=(51,31)

    start_node=list(map(int,input("Enter the start node (x y)").split()))
    start=Node(*(x for x in start_node))
    goal_node=list(map(int,input("Enter the goal node (x y)").split()))
    goal=Node(*(x for x in goal_node))

    grid = Graph(grid_size)
    plot = Visualize(start,goal,grid.obstacle_points)
    if planner == "PRM":
        gridPRM = PRM.PRM(grid,start,goal)
        grid = gridPRM.generate_PRM()

        astar = Astar.Astar(start,goal,grid)
        shortest_path,expl_nodes = astar.plan()
        plot.draw_graph(grid)

    elif planner == "Astar":
        astar = Astar.Astar(start,goal,grid)
        shortest_path,expl_nodes = astar.plan()

    elif planner == "BiAstar":
        Biastar = BidirectionalAstar.BidirectionalAstar(start,goal,grid)
        shortest_path,expl_nodes_frd,expl_nodes_bck = Biastar.plan()
        plot.animate_bi(expl_nodes_frd,expl_nodes_bck,shortest_path)

    else:
        algorithm = Dijkstra.Dijkstra(start,goal,grid)
        shortest_path,expl_nodes = algorithm.plan()
    
    # plot.animate(expl_nodes_frd,shortest_path)
    # shortest_path,expl_nodes = astar.plan()
