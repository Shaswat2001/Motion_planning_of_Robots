#!/usr/bin/env python3

import math
import numpy as np
from utils.utils import theta2RMatrix,mod2pi,getTmatrix


# def mod2pi():
#     pass

class DubinsPath:

    def __init__(self,start,goal,turning_radius,path_type):

        self.start = start
        self.goal = goal
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
    
    def compute_dubins_path(self):

        Tgoal = self.transform_coordinates(self.start,self.goal)

        d1,d2,d3,mode = self.compute_dubins_path_from_origin(Tgoal)
        return d1,d2,d3,mode

    def transform_coordinates(self,start,goal):

        Rmatrix = theta2RMatrix(start[-1])
        Tmat =  getTmatrix(Rmatrix,np.array([[start[0],start[1],1]]).T)
        print(f"{np.linalg.inv(Tmat).shape}")
        transformed_goal = np.linalg.inv(Tmat)@np.array([[goal[0],goal[1],0,1]]).T

        Tgoal = [transformed_goal[0][0],transformed_goal[1][0],goal[-1]-start[-1]]
        print(f"Transformed goal : {Tgoal}")

        return Tgoal

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
                best_cost = cost
                best_mode,best_d1,best_d2,best_d3 = mode,d1,d2,d3

        
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

if __name__ == "__main__":

    dubinsPath = DubinsPath([1,1,np.deg2rad(45)],[0,0,np.deg2rad(-45)],1,None)
    d1,d2,d3,mode = dubinsPath.compute_dubins_path()

    print(f"mode : {mode}, d1 = {d1}, d2 = {d2}, d3 = {d3}")


