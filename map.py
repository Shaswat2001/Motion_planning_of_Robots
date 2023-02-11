from Nodes import Node
import numpy as np

# list of obstacle points
class Map:

    def __init__(self,grid_size,obstacle_points = None):

        self.grid_size = grid_size
        if obstacle_points == None:
            self.obstacle_points = self.create_obstacles()
        else:
            self.obstacle_points = obstacle_points
        
        self.load_map()

    def create_obstacles(self):
        '''
        Returns a list of obstacle nodes
        '''

        # Make sure global variables are used
        obstacle_points = []

        x = self.grid_size[0]
        y = self.grid_size[1]

        for i in range(x):
            node = Node(i,0)
            obstacle_points.append(node)
        for i in range(x):
            node = Node(i,y-1)
            obstacle_points.append(node)

        for i in range(y):
            node = Node(0,i)
            obstacle_points.append(node)
        for i in range(y):
            node = Node(x-1,i)
            obstacle_points.append(node)

        for i in range(10, 21):
            node = Node(i, 15)
            obstacle_points.append(node)
        for i in range(15):
            node = Node(20, i)
            obstacle_points.append(node)

        for i in range(15, 30):
            node = Node(30, i)
            obstacle_points.append(node)
        for i in range(16):
            node = Node(40, i)
            obstacle_points.append(node)

        return obstacle_points
    
    def load_map(self):
        '''
        Created and Add obstacle points in the canvas
        '''
        # An array representing the grid
        self.maze_canvas=np.full((self.grid_size[0]+1,self.grid_size[1]+1,3),(255,255,255),dtype=np.uint8)
        # If their are Obstacles in the grid
        if len(self.obstacle_points)>0:
            # Loops through all the obstacle points
            for nodes in self.obstacle_points:
                # the colormap of the obstacle nodes is set to be (0,0,0) i.e black
                self.maze_canvas[nodes.x][nodes.y]=[0,0,0]

    def check_obstacleNode_canvas(self,node):
        '''
        Checks if particular node is in an obstacle
        '''
        # If the colormap at the node is (0,0,0) i.e black
        if (self.maze_canvas[node.x][node.y]==[0,0,0]).all():
            return True
        else:
            return False
