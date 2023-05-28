#!/usr/bin/env python3

import math
import numpy as np
from utils.make_graph import Visualize
from utils.Nodes import Node
from utils.utils import theta2RMatrix,mod2pi,getTmatrix
import matplotlib.pyplot as plt


# def mod2pi():
#     pass

class DubinsPath:

    def __init__(self,turning_radius,path_type):

        self.turning_radius = turning_radius
        self.path_type = path_type

        self.dubins_list = {"RLR":self.RLR,
                            "LRL":self.LRL,
                            "RSR":self.RSR,
                            "LSL":self.LSL,
                            "LSR":self.LSR,
                            "RSL":self.RSL}

    def get_angles(self,a,b):

        sinb = math.sin(b)
        sina = math.sin(a)
        cosa = math.cos(a)
        cosb = math.cos(b)
        cosab = math.cos(a-b)

        return sina,sinb,cosa,cosb,cosab
    
    def compute_dubins_path(self,start,goal):

        Tgoal,Tmat = self.transform_coordinates(start,goal)

        d1,d2,d3,mode = self.compute_dubins_path_from_origin(Tgoal)

        lengths = [d1,d2,d3]

        x_list,y_list,theta_list = self.generate_path_nodes(lengths,mode)
        global_x,global_y,global_theta = self.convert_nodes_global(start,x_list,y_list,theta_list,Tmat)

        # print(f"X list : {global_cord}")
        return global_x,global_y,global_theta,mode

    def transform_coordinates(self,start,goal):

        Rmatrix = theta2RMatrix(start.theta)
        Tmat =  getTmatrix(Rmatrix,np.array([[start.x,start.y,1]]).T)
        transformed_goal = np.linalg.inv(Tmat)@np.array([[goal.x,goal.y,0,1]]).T

        Tgoal = [transformed_goal[0][0],transformed_goal[1][0],goal.theta-start.theta]
        print(f"Transformed goal : {Tgoal}")

        return Tgoal,Tmat
    
    def convert_nodes_global(self,start,x_list,y_list,theta_list,Tmat):
        
        cordinates = np.array([[x,y,0,1] for (x,y) in zip(x_list,y_list)])
        global_cord = Tmat@cordinates.T

        global_x = global_cord[0][:]
        global_y = global_cord[1][:]
        global_theta = np.array(theta_list) + start.theta
        
        return global_x,global_y,global_theta
    
    def compute_dubins_path_from_origin(self,goal):

        theta = math.atan2(goal[1],goal[0])
        D = np.hypot(goal[0],goal[1])

        alpha = mod2pi(-theta)
        beta = mod2pi(goal[-1] - theta)

        if self.path_type:
            d1,d2,d3,mode = self.dubins_list[self.path_type](alpha,beta,D,self.turning_radius)
            return d1,d2,d3,mode
        
        best_mode = None
        best_cost = math.inf

        for _,path_func in self.dubins_list.items():

            d1,d2,d3,mode = path_func(alpha,beta,D,self.turning_radius)
            print(f"mode : {mode}, d1 = {d1}, d2 = {d2}, d3 = {d3}")
            if d1 == None:
                continue

            cost = d1 + d2 +d3

            if cost < best_cost:
                best_mode,best_d1,best_d2,best_d3,best_cost = mode,d1,d2,d3,cost
        
        return best_d1,best_d2,best_d3,best_mode

    def RSR(self,alpha,beta,D,rad):

        sina,sinb,cosa,cosb,cosab = self.get_angles(alpha,beta)
        mode = ["R","S","R"]
        d_squared = D**2 + 2*(rad**2) - 2*cosab*(rad**2) + 2*rad*D*(sinb - sina)
        temp = math.atan2(rad*(cosa - cosb),D + rad*(-sina + sinb))

        if d_squared < 0:
            return None,None,None,mode

        d1 = mod2pi(alpha - temp)
        d2 = math.sqrt(d_squared)
        d3 = mod2pi(-beta + temp)

        return d1,d2,d3,mode

    def LSL(self,alpha,beta,D,rad):

        sina,sinb,cosa,cosb,cosab = self.get_angles(alpha,beta)
        mode = ["L","S","L"]
        d_squared = D**2 + 2*(rad**2) - 2*cosab*(rad**2) + 2*rad*D*(sina - sinb)
        temp = math.atan2(rad*(cosb - cosa),D + rad*(sina - sinb))

        if d_squared < 0:
            return None,None,None,mode

        d1 = mod2pi(-alpha + temp)
        d2 = math.sqrt(d_squared)
        d3 = mod2pi(beta - temp)

        return d1,d2,d3,mode

    def LSR(self,alpha,beta,D,rad):

        sina,sinb,cosa,cosb,cosab = self.get_angles(alpha,beta)
        mode = ["L","S","R"]
        d_squared = D**2 - 2*(rad**2) + 2*cosab*(rad**2) + 2*rad*D*(sina + sinb)
        
        if d_squared < 0:
            return None,None,None,mode
        
        d2 = math.sqrt(d_squared)

        temp = math.atan2(-rad*(cosb + cosa),D + rad*(sina + sinb)) - math.atan2(-2,d2)
        d1 = mod2pi(-alpha + temp)
        d3 = mod2pi(-mod2pi(beta) + temp)

        return d1,d2,d3,mode

    def RSL(self,alpha,beta,D,rad):

        sina,sinb,cosa,cosb,cosab = self.get_angles(alpha,beta)
        mode = ["R","S","L"]
        d_squared = D**2 - 2*(rad**2) + 2*cosab*(rad**2) - 2*rad*D*(sina + sinb)
        
        if d_squared < 0:
            return None,None,None,mode
        
        d2 = math.sqrt(d_squared)
        
        temp = math.atan2(rad*(cosb + cosa),D - rad*(sina + sinb)) - math.atan2(2,d2)
        d1 = mod2pi(alpha - temp)
        d3 = mod2pi(beta - temp)

        return d1,d2,d3,mode

    def RLR(self,alpha,beta,D,rad):

        sina,sinb,cosa,cosb,cosab = self.get_angles(alpha,beta)
        mode = ["R","L","R"]
        temp = (6*(rad**2) - D**2 + 2*cosab*(rad**2) + 2*rad*D*(sina - sinb))/8
        
        if abs(temp) > 1:
            return None,None,None,mode
        
        d2 = mod2pi(2*math.pi - math.acos(temp))        
        d1 = mod2pi(alpha - math.atan2(rad*(cosa - cosb), D - rad*(-sina + sinb)) + d2 / 2.0)
        d3 = mod2pi(alpha - beta - d1 + d2)

        return d1,d2,d3,mode

    def LRL(self,alpha,beta,D,rad):

        sina,sinb,cosa,cosb,cosab = self.get_angles(alpha,beta)
        mode = ["L","R","L"]
        temp = (6*(rad**2) - D**2 + 2*cosab*(rad**2) + 2*rad*D*(sinb - sina))/8
        
        if abs(temp) > 1:
            return None,None,None,mode
        
        d2 = mod2pi(2*math.pi - math.acos(temp))        
        d1 = mod2pi(-alpha - math.atan2(rad*(cosa - cosb), D + rad*(sina - sinb)) + d2 / 2.0)
        d3 = mod2pi(-alpha + mod2pi(beta) - d1 + mod2pi(d2))

        return d1,d2,d3,mode
    
    def generate_path_nodes(self,mode_lengths,mode,step_size = 0.1):

        d_x,d_y,d_theta = [0.0],[0.0],[0.0]

        for dir,length in zip(mode,mode_lengths):

            if length == 0:
                continue
            
            origin_x,origin_y,origin_theta = d_x[-1],d_y[-1],d_theta[-1]

            current_length = step_size

            while abs(current_length + step_size) <= abs(length):
                d_x,d_y,d_theta = self.interpolate_section(current_length,dir,origin_x,origin_y,origin_theta,d_x,d_y,d_theta)

                current_length += step_size

            d_x,d_y,d_theta = self.interpolate_section(length,dir,origin_x,origin_y,origin_theta,d_x,d_y,d_theta)

        return d_x,d_y,d_theta
        
    def interpolate_section(self,length,mode,x,y,theta,d_x,d_y,d_theta):

        if mode == "S":
            d_x.append(x + length*self.turning_radius*math.cos(theta))
            d_y.append(y + length*self.turning_radius*math.sin(theta))
            d_theta.append(theta)

        else:

            ldx = math.sin(length)*self.turning_radius
            ldy = 0

            if mode == "L":
                ldy = (1 - math.cos(length))*self.turning_radius
            elif mode == "R":
                ldy = -(1 - math.cos(length))*self.turning_radius

            gdx = math.cos(-theta) * ldx + math.sin(-theta) * ldy
            gdy = -math.sin(-theta) * ldx + math.cos(-theta) * ldy
            d_x.append(x + gdx)
            d_y.append(y + gdy)

            if mode == "L":  # left turn
                d_theta.append(theta + length)
            elif mode == "R":  # right turn
                d_theta.append(theta - length)

        return d_x,d_y,d_theta

if __name__ == "__main__":

    start = Node(1,1,np.deg2rad(45))
    goal = Node(-3,-3,np.deg2rad(-45))

    dubinsPath = DubinsPath(1,None)
    global_x, global_y,global_theta,mode = dubinsPath.compute_dubins_path(start,goal)
    
    plot_result = Visualize(start[0:2],goal[0:2],None,[0,0],None)
    plot_result.plot_curve(global_x, global_y,global_theta,mode)


