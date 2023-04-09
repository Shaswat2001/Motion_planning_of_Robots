import math
import itertools
from utils.Nodes import Node

class NonHolonomicDrive:

    def __init__(self,RPM,grid_size,r = 3.8,L = 35,time_run = 1):

        self.RPM_L = RPM[0]
        self.RPM_R = RPM[1]
        self.grid_size = grid_size
        self.r = 3.8
        self.L = 35
        self.time_run = 1

    def get_neighbours(self,current_node,current_orientation):

        all_neighbours = {}

        RPM_list = list(itertools.permutations([0,self.RPM_L,self.RPM_R],2))
        RPM_list.append((self.RPM_R,self.RPM_R))
        RPM_list.append((self.RPM_L,self.RPM_L))

        for RPM in RPM_list:

            nbr,_ = self.MoveRobot(current_node,current_orientation,RPM[0],RPM[1])

            if self.grid_size[0][0] <= nbr[2].x <= self.grid_size[0][1] and self.grid_size[1][0] <= nbr[2].y <= self.grid_size[1][1]:

                all_neighbours[nbr[2]] = (nbr[0],nbr[1])

        return all_neighbours

    def MoveRobot(self,current_node,current_orientation,RPM_L,RPM_R):

        current_t = 0
        dt = 0.1
        initial_theta = 3.14 * current_orientation / 180.0

        uL = self.r*RPM_L*0.10472
        uR = self.r*RPM_R*0.10472
        new_x = current_node.x
        new_y = current_node.y
        while current_t<self.time_run:
            current_t+= dt
            new_x += (self.r/2.0)*(uL+uR)*math.cos(initial_theta)*dt
            new_y += (self.r/2.0)*(uL+uR)*math.sin(initial_theta)*dt
            initial_theta += (self.r/self.L)*(uR-uL)*dt

        final_theta = 180*initial_theta/3.14
        degree_rotated = abs(current_orientation - final_theta)
        degree_rotated = abs(degree_rotated % 360)
        cost = 10 + (10*degree_rotated/360.0)

        new_x = self.roundBaseTwo(new_x)
        new_y = self.roundBaseTwo(new_y)

        new_node = (cost,self.roundToFifeteen(final_theta),Node(new_x,new_y))

        if self.grid_size[0][0] <= new_x <= self.grid_size[0][1] and self.grid_size[1][0] <= new_y <= self.grid_size[1][1]:

            return new_node,True
        else:
            return (100000,current_orientation,current_node)
        
    def roundBaseTwo(self,node):

        return (round(node*2)/2.0)

    def roundToFifeteen(self,node,base = 15.0):

        return base*round(node/base)





