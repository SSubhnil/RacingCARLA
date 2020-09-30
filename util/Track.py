
import numpy as np
import numpy.linalg as la
import pdb
from sympy import *


class Map:
    """map object
    Attributes:
        getGlobalPosition: convert position from (s, ey) to (X,Y)
    """
    def __init__(self, halfwidth, data):
        """Initialization
        halfWidth: track halfWidth
        Modify the vector spec to change the geometry of the track
        """
        self.count = -1
        self.Acount = -1
        self.halfWidth = halfwidth
        self.slack = 0.45
        self.dist = 0.0
        self.Adist = 0.0 #Auxiliary Distance
        self.PointAndTangent = data
        self.TrackLength = 0.0
        
        

    # def create(self, data, LapTrigger):
        
    #     self.Acount += 1
    #     self.count += 1            
        
    #     if LapTrigger == 0:
    #         self.dist = self.dist + data[0, 3]
    #         self.Adist = self.Adist + data[0, 3]
            
    #         self.auxiliary[self.Acount,0] = data[0, 0]
    #         self.auxiliary[self.Acount,1] = data[0, 1]
    #         self.auxiliary[self.Acount,2] = data[0, 2]
    #         self.auxiliary[self.Acount,3] = self.Adist
            
    #         self.PointAndTangent[self.count,0] = self.auxiliary[self.Acount,0]
    #         self.PointAndTangent[self.count,1] = self.auxiliary[self.Acount,1]
    #         self.PointAndTangent[self.count,2] = self.auxiliary[self.Acount,2]
    #         self.PointAndTangent[self.count,3] = self.dist
            
    #     else:
    #         self.dist = 0
    #         self.count = -1
        
        
            
        
    def StartComputation(self):
        
        bracket = 50
        #FInd the finish line from the data
        for i in range(50, len(self.PointAndTangent)-55):
            upper_lim = i + bracket
            lower_lim = i- bracket
            roi_x = self.PointAndTangent[lower_lim:upper_lim, 0]
            roi_y = self.PointAndTangent[lower_lim:upper_lim, 1]
            self.PointAndTangent[i, 4] = Map.ComputeCurvature(roi_x, roi_y, self.PointAndTangent[i,0])
        
        
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
            
        
            
        self.TrackLength = self.PointAndTangent[-1,3]
        np.save('pnt_curv',self.PointAndTangent)
        return True
        
    def ComputeCurvature(x, y, ref_point):
        
        const = np.polyfit(x,y,3)
        X = Symbol('X')
        fun = const[0]*X**3 + const[1]*X**2 + const[2]*X + const[3] 
        
        first_deriv = Derivative(fun,X)
        second_deriv = Derivative(first_deriv, X)
        
        Y_dash = first_deriv.doit().subs({X:ref_point})
        Y_Ddash= second_deriv.doit().subs({X:ref_point})
        
        curvature = Y_Ddash/pow(1+pow(Y_dash,2),3/2)
        
        return curvature
        
    def Update(self, data):
        self.PointAndTangent = data
        
        
#######################
#SS: Dont need this section. Save actual coordinate position of the car.
#SS: Save ey, s, 
#######################

    def getGlobalPosition(self, s, ey):
        """coordinate transformation from curvilinear reference frame (s, ey) to inertial reference frame (X, Y)
        (s, ey): position in the curvilinear reference frame
        """

        # wrap s along the track
        while (s > self.TrackLength):
            s = s - self.TrackLength

        # Compute the segment in which system is evolving
        PointAndTangent = self.PointAndTangent

        index = np.all([[s >= PointAndTangent[:, 3]], [s < PointAndTangent[:, 3] + PointAndTangent[:, 4]]], axis=0)
        i = int(np.where(np.squeeze(index))[0])

        if PointAndTangent[i, 5] == 0.0:  # If segment is a straight line
            # Extract the first final and initial point of the segment
            xf = PointAndTangent[i, 0]
            yf = PointAndTangent[i, 1]
            xs = PointAndTangent[i - 1, 0]
            ys = PointAndTangent[i - 1, 1]
            psi = PointAndTangent[i, 2]

            # Compute the segment length
            deltaL = PointAndTangent[i, 4]
            reltaL = s - PointAndTangent[i, 3]

            # Do the linear combination
            x = (1 - reltaL / deltaL) * xs + reltaL / deltaL * xf + ey * np.cos(psi + np.pi / 2)
            y = (1 - reltaL / deltaL) * ys + reltaL / deltaL * yf + ey * np.sin(psi + np.pi / 2)
        else:
            r = 1 / PointAndTangent[i, 5]  # Extract curvature
            ang = PointAndTangent[i - 1, 2]  # Extract angle of the tangent at the initial point (i-1)
            # Compute the center of the arc
            if r >= 0:
                direction = 1
            else:
                direction = -1

            CenterX = PointAndTangent[i - 1, 0] \
                      + np.abs(r) * np.cos(ang + direction * np.pi / 2)  # x coordinate center of circle
            CenterY = PointAndTangent[i - 1, 1] \
                      + np.abs(r) * np.sin(ang + direction * np.pi / 2)  # y coordinate center of circle

            spanAng = (s - PointAndTangent[i, 3]) / (np.pi * np.abs(r)) * np.pi

            angleNormal = wrap((direction * np.pi / 2 + ang))
            angle = -(np.pi - np.abs(angleNormal)) * (sign(angleNormal))

            x = CenterX + (np.abs(r) - direction * ey) * np.cos(
                angle + direction * spanAng)  # x coordinate of the last point of the segment
            y = CenterY + (np.abs(r) - direction * ey) * np.sin(
                angle + direction * spanAng)  # y coordinate of the last point of the segment

        return x, y


    def getLocalPosition(self, x, y, psi):
        """coordinate transformation from inertial reference frame (X, Y) to curvilinear reference frame (s, ey)
        (X, Y): position in the inertial reference frame
        """
        PointAndTangent = self.PointAndTangent
        CompletedFlag = 0



        for i in range(0, PointAndTangent.shape[0]):
            if CompletedFlag == 1:
                break

            if PointAndTangent[i, 5] == 0.0:  # If segment is a straight line
                # Extract the first final and initial point of the segment
                xf = PointAndTangent[i, 0]
                yf = PointAndTangent[i, 1]
                xs = PointAndTangent[i - 1, 0]
                ys = PointAndTangent[i - 1, 1]

                psi_unwrap = np.unwrap([PointAndTangent[i - 1, 2], psi])[1]
                epsi = psi_unwrap - PointAndTangent[i - 1, 2]
                # Check if on the segment using angles
                if (la.norm(np.array([xs, ys]) - np.array([x, y]))) == 0:
                    s  = PointAndTangent[i, 3]
                    ey = 0
                    CompletedFlag = 1

                elif (la.norm(np.array([xf, yf]) - np.array([x, y]))) == 0:
                    s = PointAndTangent[i, 3] + PointAndTangent[i, 4]
                    ey = 0
                    CompletedFlag = 1
                else:
                    if np.abs(computeAngle( [x,y] , [xs, ys], [xf, yf])) <= np.pi/2 and np.abs(computeAngle( [x,y] , [xf, yf], [xs, ys])) <= np.pi/2:
                        v1 = np.array([x,y]) - np.array([xs, ys])
                        angle = computeAngle( [xf,yf] , [xs, ys], [x, y])
                        s_local = la.norm(v1) * np.cos(angle)
                        s       = s_local + PointAndTangent[i, 3]
                        ey      = la.norm(v1) * np.sin(angle)

                        if np.abs(ey)<= self.halfWidth + self.slack:
                            CompletedFlag = 1

            else:
                xf = PointAndTangent[i, 0]
                yf = PointAndTangent[i, 1]
                xs = PointAndTangent[i - 1, 0]
                ys = PointAndTangent[i - 1, 1]

                r = 1 / PointAndTangent[i, 5]  # Extract curvature
                if r >= 0:
                    direction = 1
                else:
                    direction = -1

                ang = PointAndTangent[i - 1, 2]  # Extract angle of the tangent at the initial point (i-1)

                # Compute the center of the arc
                CenterX = xs + np.abs(r) * np.cos(ang + direction * np.pi / 2)  # x coordinate center of circle
                CenterY = ys + np.abs(r) * np.sin(ang + direction * np.pi / 2)  # y coordinate center of circle

                # Check if on the segment using angles
                if (la.norm(np.array([xs, ys]) - np.array([x, y]))) == 0:
                    ey = 0
                    psi_unwrap = np.unwrap([ang, psi])[1]
                    epsi = psi_unwrap - ang
                    s = PointAndTangent[i, 3]
                    CompletedFlag = 1
                elif (la.norm(np.array([xf, yf]) - np.array([x, y]))) == 0:
                    s = PointAndTangent[i, 3] + PointAndTangent[i, 4]
                    ey = 0
                    psi_unwrap = np.unwrap([PointAndTangent[i, 2], psi])[1]
                    epsi = psi_unwrap - PointAndTangent[i, 2]
                    CompletedFlag = 1
                else:
                    arc1 = PointAndTangent[i, 4] * PointAndTangent[i, 5]
                    arc2 = computeAngle([xs, ys], [CenterX, CenterY], [x, y])
                    if np.sign(arc1) == np.sign(arc2) and np.abs(arc1) >= np.abs(arc2):
                        v = np.array([x, y]) - np.array([CenterX, CenterY])
                        s_local = np.abs(arc2)*np.abs(r)
                        s    = s_local + PointAndTangent[i, 3]
                        ey   = -np.sign(direction) * (la.norm(v) - np.abs(r))
                        psi_unwrap = np.unwrap([ang + arc2, psi])[1]
                        epsi = psi_unwrap - (ang + arc2)

                        if np.abs(ey) <= self.halfWidth + self.slack:
                            CompletedFlag = 1

        if epsi>1.0:
            pdb.set_trace()

        if CompletedFlag == 0:
            s    = 10000
            ey   = 10000
            epsi = 10000

            print("Error!! POINT OUT OF THE TRACK!!!! <==================")
            pdb.set_trace()

        return s, ey, epsi, CompletedFlag

# ======================================================================================================================
# ======================================================================================================================
# ====================================== Internal utilities functions ==================================================
# ======================================================================================================================
# ======================================================================================================================
def computeAngle(point1, origin, point2):
    # The orientation of this angle matches that of the coordinate system. Tha is why a minus sign is needed
    v1 = np.array(point1) - np.array(origin)
    v2 = np.array(point2) - np.array(origin)
    #
    # cosang = np.dot(v1, v2)
    # sinang = la.norm(np.cross(v1, v2))
    #
    # dp = np.dot(v1, v2)
    # laa = la.norm(v1)
    # lba = la.norm(v2)
    # costheta = dp / (laa * lba)

    dot = v1[0] * v2[0] + v1[1] * v2[1]  # dot product between [x1, y1] and [x2, y2]
    det = v1[0] * v2[1] - v1[1] * v2[0]  # determinant
    angle = np.arctan2(det, dot)  # atan2(y, x) or atan2(sin, cos)

    return angle # np.arctan2(sinang, cosang)

def wrap(angle):
    if angle < -np.pi:
        w_angle = 2 * np.pi + angle
    elif angle > np.pi:
        w_angle = angle - 2 * np.pi
    else:
        w_angle = angle

    return w_angle

def sign(a):
    if a >= 0:
        res = 1
    else:
        res = -1

    return res

def unityTestChangeOfCoordinates(map, ClosedLoopData):
    """For each point in ClosedLoopData change (X, Y) into (s, ey) and back to (X, Y) to check accurancy
    """
    TestResult = 1
    for i in range(0, ClosedLoopData.x.shape[0]):
        xdat = ClosedLoopData.x
        xglobdat = ClosedLoopData.x_glob

        s, ey, _, _ = map.getLocalPosition(xglobdat[i, 4], xglobdat[i, 5], xglobdat[i, 3])
        v1 = np.array([s, ey])
        v2 = np.array(xdat[i, 4:6])
        v3 = np.array(map.getGlobalPosition(v1[0], v1[1]))
        v4 = np.array([xglobdat[i, 4], xglobdat[i, 5]])
        # print v1, v2, np.dot(v1 - v2, v1 - v2), np.dot(v3 - v4, v3 - v4)

        if np.dot(v3 - v4, v3 - v4) > 0.00000001:
            TestResult = 0
            print("ERROR", v1, v2, v3, v4)
            pdb.set_trace()
            v1 = np.array(map.getLocalPosition(xglobdat[i, 4], xglobdat[i, 5]))
            v2 = np.array(xdat[i, 4:6])
            v3 = np.array(map.getGlobalPosition(v1[0], v1[1]))
            v4 = np.array([xglobdat[i, 4], xglobdat[i, 5]])
            print(np.dot(v3 - v4, v3 - v4))
            pdb.set_trace()

    if TestResult == 1:
        print("Change of coordinates test passed!")