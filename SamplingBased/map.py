from Nodes import Node,check_NodeIn_list,check_nodes
import numpy as np

# list of obstacle points
class Map:

    def __init__(self,grid_size):

        self.grid_size = grid_size
        self.obs_boundary = self.obs_boundary()
        self.obs_rectangle = self.obs_rectangle()
        self.obs_circle = self.obs_circle()


    @staticmethod
    def obs_boundary():

        obs_bound = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]

        return obs_bound
    
    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [14, 12, 8, 2],
            [18, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3]
        ]

        return obs_cir
        
    
    def create_obstacles(self):
        '''
        Returns a list of obstacle nodes
        '''

        # Make sure global variables are used
        obstacle_points = []

        for x in range(self.grid_size[0]+1):
            for y in range(self.grid_size[1]+1):
                if(130+x>=y) and (290-7*x<=y) and ((17/3)*x-90<=y):
                    node=Node(x,y)
                    obstacle_points.append(node)
                # Complex shaped Obstacle
                if (x>=90 and 5*x-360<=y and y<=155) or (x>=90 and(x+530>=4*y) and ((5/6)*x+(170/3)<=y) and x<=130):
                    node=Node(x,y)
                    obstacle_points.append(node)
                # Complex shaped Obstacle
                if x>=120 and x<=160 and y>=35 and y<=130:
                    if (x-10)>=y:
                        if x-400<=-2*y:
                            if 3*x-360<=y:
                                if x-60<=y or (-7/3)*x+(1120/3)>=y:
                                    if (-2/5)*x +93<=y:
                                        node=Node(x,y)
                                        obstacle_points.append(node)
                # Triangular Shaped Obstacle
                if (2*x-340>=y) and ((-5/2)*x+605>=y) and (x-350>=-4*y):
                    node=Node(x,y)
                    obstacle_points.append(node)
                # Trapezoidal Shaped Obstacle
                if (-3*x+960>=y) and ((2/11)*x+(1460/11)>=y) and ((7/2)*x-(565)>=y) and (x+580<=5*y):
                    node=Node(x,y)
                    obstacle_points.append(node)

        return obstacle_points

    def check_obstacleNode_canvas(self,node):
        '''
        Checks if particular node is in an obstacle
        '''
        # If the node is present in the list of obstacle points
        if check_NodeIn_list(node,self.obstacle_points):
            return True
        else:
            return False
