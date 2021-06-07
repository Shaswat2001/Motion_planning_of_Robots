start=list(map(int,input("Enter the Start Node (x,y)").split()))
goal=list(map(int,input("Enter the Goal Node (x,y)").split()))

algorithm=input("Enter the algorithm you want to run")

if algorithm=='A_star':
    doAstar_algorithm(start,goal)
elif algorithm=='Dijkstra':
    doDijkstra_algorithm(start,goal)
else:
    print("Invalid Input!! Try Again")
