import numpy as np
import math

def mod2pi(angle,range_zero_2pi = True):

    if range_zero_2pi:
        return angle%(2*np.pi)
    else:
        pass

def theta2RMatrix(angle,radians = True):

    if not radians:

        angle = np.deg2rad(angle)

    ctheta = math.cos(angle)
    stheta = math.sin(angle)
    Rmat = np.array([[ctheta, -stheta, 0],
                     [stheta,  ctheta, 0],
                     [0, 0, 1]])
    
    return Rmat

def getTmatrix(Rmat,pmat):

    Tmat = np.concatenate((Rmat,pmat),axis=-1)
    Tmat = np.concatenate((Tmat,np.array([[0,0,0,1]])))

    return Tmat

if __name__ == "__main__":

    angle = 0.523

    R = theta2RMatrix(angle)

    angle = 30

    print(theta2RMatrix(angle,False))

    print(getTmatrix(R,np.array([[0,3,4]]).T))

    