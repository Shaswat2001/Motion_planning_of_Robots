from Nodes import Node,check_NodeIn_list
import numpy as np

# list of obstacle points
class Map:

    def __init__(self,grid_size,obstacle_points = None):

        self.grid_size = grid_size
        if obstacle_points == None:
            self.obstacle_points = self.create_obstacles()
        else:
            self.obstacle_points = obstacle_points

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
    
    def update_obsMap(self,obsNode):

        if self.obstacle_points == None:
            self.obstacle_points = [obsNode]
            return self.obstacle_points

        self.obstacle_points.append(obsNode)
        return self.obstacle_points

    def check_obstacleNode_canvas(self,node):
        '''
        Checks if particular node is in an obstacle
        '''
        # If the node is present in the list of obstacle points
        if check_NodeIn_list(node,self.obstacle_points):
            return True
        else:
            return False
