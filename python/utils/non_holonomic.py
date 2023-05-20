import math
import itertools
from utils.Nodes import Node

class NonHolonomicDrive:

    def __init__(self,RPM,grid_size,r = 3.8,L = 35,time_run = 1):

        self.RPM_L = RPM[0]
        self.RPM_R = RPM[1]
        self.grid_size = grid_size
        self.r = r
        self.L = L
        self.time_run = time_run

    def get_neighbours(self,current_node):

        all_neighbours = {}

        RPM_list = list(itertools.permutations([0,self.RPM_L,self.RPM_R],2))
        RPM_list.append((self.RPM_R,self.RPM_R))
        RPM_list.append((self.RPM_L,self.RPM_L))

        for RPM in RPM_list:

            nbr,_,int_nodes = self.MoveRobot(current_node,RPM[0],RPM[1])
            robot_twist = self.convert_rpm_to_robospeed(RPM,self.r,self.L)
            
            if self.grid_size[0][0] <= nbr[1].x <= self.grid_size[0][1] and self.grid_size[1][0] <= nbr[1].y <= self.grid_size[1][1]:
                all_neighbours[nbr[1]] = {"cost":nbr[0],
                                          "twist":robot_twist,
                                          "intermediate_nodes":int_nodes}
        
        return all_neighbours

    def MoveRobot(self,current_node,RPM_L,RPM_R):

        current_t = 0
        dt = 0.1
        intermediate_nodes = []
        initial_theta = current_node.theta

        uL = self.r*RPM_L*0.10472
        uR = self.r*RPM_R*0.10472
        new_x = current_node.x
        new_y = current_node.y
        while current_t<self.time_run:
            current_t+= dt
            new_x += (self.r/2.0)*(uL+uR)*math.cos(initial_theta)*dt
            new_y += (self.r/2.0)*(uL+uR)*math.sin(initial_theta)*dt
            initial_theta += (self.r/self.L)*(uR-uL)*dt
            intermediate_nodes.append([new_x,new_y,initial_theta])

        initial_theta = initial_theta%(2*math.pi)
        degree_rotated = abs(current_node.theta - initial_theta)
        degree_rotated = abs(degree_rotated % (2*math.pi))
        cost = 10 + (10*degree_rotated/(2*math.pi))

        new_x = self.roundBaseTwo(new_x)
        new_y = self.roundBaseTwo(new_y)

        new_node = (cost,Node(new_x,new_y,initial_theta))

        if self.grid_size[0][0] <= new_x <= self.grid_size[0][1] and self.grid_size[1][0] <= new_y <= self.grid_size[1][1]:

            return new_node,True,intermediate_nodes
        else:
            return (100000,current_node),False,intermediate_nodes
    
    def convert_rpm_to_robospeed(self,RPM,r,l):

        wheel_vel = [2*math.pi*N/60 for N in RPM]

        omega = r*(wheel_vel[1] - wheel_vel[0])/l
        linear_vel = sum(wheel_vel)*r/(2*100)

        return [omega,linear_vel]

    def roundBaseTwo(self,node):

        return (round(node*2)/2.0)

def roundToBase(node,base = 15):

    return base*round(node/base)


if __name__ == "__main__":

    drive = NonHolonomicDrive([20,30],[[-510,510],[-510,510]])

    print(drive.get_neighbours(Node(-430,-300,0)))




