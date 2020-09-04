import matplotlib.pyplot as plt

import numpy as np
import numpy.linalg as la
import pdb
from sympy import *
import math

def ComputeCurvature(x, y, ref_point):
    const = np.polyfit(x,y,2)
    X = Symbol('X')
    fun = const[0]*X**2 + const[1]*X + const[2] 
    
    first_deriv = Derivative(fun,X)
    second_deriv = Derivative(first_deriv, X)
    
    Y_dash = first_deriv.doit().subs({X:ref_point})
    Y_Ddash= second_deriv.doit().subs({X:ref_point})
    
    curvature = Y_Ddash/pow(1+pow(Y_dash,2),3/2)

    return curvature

def isInside(x, y): 
      
    # Compare radius of circle 
    # with distance of its center 
    # from given point 
    if (pow(x - 4.099, 2) + pow(y - -12.9193, 2) <= pow(11,2)): 
        rad = -math.sqrt(pow(x - 4.099, 2) + pow(y - -12.9193, 2))
    
    elif (pow(x - 9.7842, 2) + pow(y - 54.5053, 2) <= pow(63,2)):
        rad = math.sqrt(pow(x - 9.7842, 2) + pow(y - 54.5053, 2))
        
    elif (pow(x - 63.8352, 2) + pow(y - -7.2731, 2) <= pow(9,2)):
        rad = -math.sqrt(pow(x - 63.8352, 2) + pow(y - -7.2731, 2))
    
    elif (pow(x - 54.5997, 2) + pow(y - -23.5583, 2) <= pow(24,2)):
        rad = -math.sqrt(pow(x - 54.5997, 2) + pow(y - -23.5583, 2))
        
    elif (pow(x - 88.1886, 2) + pow(y - -63.8471, 2) <= pow(33,2)):
        rad = math.sqrt(pow(x - 88.1886, 2) + pow(y - -63.8471, 2))
    
    elif (pow(x - 28.7949, 2) + pow(y - -118.3814, 2) <= pow(40,2)):
        rad = -math.sqrt(pow(x - 28.7949, 2) + pow(y - -118.3814, 2))
    
    elif (pow(x - 153.9146, 2) + pow(y - -193.2113, 2) <= pow(115.5,2)):
        rad = math.sqrt(pow(x - 153.9146, 2) + pow(y - -193.2113, 2))
    
    elif (pow(x - 11.6724, 2) + pow(y - -209.6244, 2) <= pow(31,2)):
        rad = -math.sqrt(pow(11.6724, 2) + pow(y - -209.6244, 2))
    
    elif (pow(x - 6.9916, 2) + pow(y - -232.2871, 2) <= pow(11.5,2)):
        rad = -math.sqrt(pow(11.6724, 2) + pow(y - -209.6244, 2))
    
    else:
        return 0.0
    
    return 1/rad

def main():
    new = np.load('pnt_curv.npy')
            
    # bracket = 350
    # #FInd the finish line from the data
    # for i in range(350, len(PointAndTangent)-350):
    #     upper_lim = i + bracket
    #     lower_lim = i- bracket
    #     roi_x = PointAndTangent[lower_lim:upper_lim, 0]
    #     roi_y = PointAndTangent[lower_lim:upper_lim, 1]
    #     PointAndTangent[i, 4] = ComputeCurvature(roi_x, roi_y, PointAndTangent[i,1])
        
    # np.save('pnt_curv_temp_X_2deg',PointAndTangent)

    # new = np.load('pnt_curv_temp_X_2deg.npy')
    
    for i in range(0, len(new)):
        new[i, 4] = isInside(new[i,0], new[i,1])
    
    np.save('pnt_new',new)
    plt.xlabel('Distance (m)')
    plt.ylabel('Curvature')
    plt.plot(new[:,3], new[:,4])
    plt.show

if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!') 
        
        



#The car stops before finish line. Copy the 
# for i in range(0, diff - 1):
#     self.auxiliary[i + APoints, 0] = self.auxiliary[i,0]
#     self.auxiliary[i + APoints, 1] = self.auxiliary[i,1]
#     self.auxiliary[i + APoints, 2] = self.auxiliary[i,2]
#     self.auxiliary[i + APoints, 3] = self.auxiliary[i,3] + self.auxiliary[APoints-1, 3]
    
#     self.PointAndTangent[i + Points, 0] = self.auxiliary[i,0]
#     self.PointAndTangent[i + Points, 1] = self.auxiliary[i,1]
#     self.PointAndTangent[i + Points, 2] = self.auxiliary[i,2]
#     self.PointAndTangent[i + Points, 3] = self.auxiliary[i,3] + self.PointsAndTangent[Points-1, 3]
    
# self.PointAndTangent = np.delete(self.PointAndTangent, slice(Points+diff-2 , 19999), axis=0) 
# self.auxiliary = np.delete(self.auxiliary, slice(Points+diff-2 , 19999), axis=0)            

        