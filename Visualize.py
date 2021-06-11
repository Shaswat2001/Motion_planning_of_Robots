import matplotlib.pyplot as plt
from map import obstacle_points,create_obstacles

def plot_graph(graph):
    global obstacle_points
    create_obstacles()
    obs_x=[]
    obs_y=[]
    for pt in obstacle_points:
        (x,y)=pt.get_coordinates()
        obs_x.append(x)
        obs_y.append(y)

    plt.scatter(obs_x,obs_y,c='blue')
    for i in graph.get_vertices():
        for j in graph.get_neighbours(i):
            i_cord=i.get_coordinates()
            j_cord=j.get_coordinates()
            plt.plot([i_cord[0],j_cord[0]],[i_cord[1],j_cord[1]],c='red')
    plt.show()
