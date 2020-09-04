import numpy as np
import pdb
import datetime
from Utilities import Curvature
import math
class Simulator:
    """Vehicle simulator
    Attributes:
        Sim: given a Controller object run closed-loop simulation and write the data in the ClosedLoopData object
    """
    def __init__(self, Dynamic, map, lap = 0, flagLMPC = 0):
        """Initialization
        map: map
        lap: number of laps to run. If set to 0 then the simulation is completed when ClosedLoopData is full
        flagLMPC: set to 0 for standart controller. Set to 1 for LMPC --> at iteration j add data to SS^{j-1} (look line 9999)
        """
        self.map = map
        self.laps = lap
        self.flagLMPC = flagLMPC
        self.i = 0
        self.u = np.array([0., 0.])
        self.Dynamic = Dynamic
    
    def reset(self):
        self.i = 0

    def Sim(self, x, x_glob, SimTime, Controller, LMPCprediction=0):
        """Simulate closed-loop system
        ClosedLoopData: object where the closed-loop data are written
        Controller: controller used in the closed-loop
        LMPCprediction: object where the open-loop predictions and safe set are stored
        """

        # Assign x = ClosedLoopData.x. IMPORTANT: x and ClosedLoopData.x share the same memory and therefore
        # if you modofy one is like modifying the other one!
        #xxxxxxxxxxxxxxxxxxxxxxxxxxxx Remove this for time steppingxxxxxxxxxxxxxxx

        
        Controller.solve(x)

        self.u = Controller.uPred

        if LMPCprediction != 0:
            Controller.LapTime = SimTime
            LMPCprediction.PredictedStates[:,:,self.i, Controller.it]   = Controller.xPred
            LMPCprediction.PredictedInputs[:, :, self.i, Controller.it] = Controller.uPred
            LMPCprediction.SSused[:, :, self.i, Controller.it]          = Controller.SS_PointSelectedTot
            LMPCprediction.Qfunused[:, self.i, Controller.it]           = Controller.Qfun_SelectedTot
            self.i += 1
        new_x_glob, new_x = self.Dynamic._DynModel(x, x_glob, self.u, np, 0.05, self.map.PointAndTangent)
        

        # print(getAngle(x[i+1,4], x[i+1,3], self.map.PointAndTangent))
        # print(x[i+1,3], x_glob[i+1,3], wrap(x_glob[i+1,3]))
        # pdb.set_trace()

        # if i <= 5:
        #     print("Linearization time: %.4fs Solver time: %.4fs" % (Controller.linearizationTime.total_seconds(), Controller.solverTime.total_seconds()))
        #     print("Time: ", i * ClosedLoopData.dt, "Current State and Input: ", x[i, :], u[i, :])

        # if Controller.feasible == 0:
        #     print("Unfeasible at time ", i*ClosedLoopData.dt)
        #     print("Cur State: ", x[i, :], "Iteration ", Controller.it)
        #     break

        if self.flagLMPC == 1:
            Controller.addPoint(x, self.u)

        
        # print("Number of laps completed: ", int(np.floor(x[-1, 4] / (self.map.TrackLength))))
        return self.u, new_x, new_x_glob
class PID:
    """Create the PID controller used for path following at constant speed
    Attributes:
        solve: given x0 computes the control action
    """
    def __init__(self, vt):
        """Initialization
        Arguments:
            vt: target velocity
        """
        self.vt = vt
        self.uPred = np.array([0., 0.])

        startTimer = datetime.datetime.now()
        endTimer = datetime.datetime.now(); deltaTimer = endTimer - startTimer
        self.solverTime = deltaTimer
        self.linearizationTime = deltaTimer
        self.feasible = 1

    def solve(self, x0):
        """Computes control action
        Arguments:
            x0: current state position
        """
        vt = self.vt
        self.uPred[0] = - 0.6 * x0[5] - 0.9 * x0[3] + np.max([-0.9, np.min([np.random.randn() * 0.25, 0.9])])
        self.uPred[1] = 1.5 * (vt - x0[0]) + np.max([-0.2, np.min([np.random.randn() * 0.10, 0.2])])
# ======================================================================================================================
# ======================================================================================================================
# ================================ Internal functions for change of coordinates ========================================
# ======================================================================================================================
# ======================================================================================================================
class DynamicModel:
    def __init__(self, physics):
        # Vehicle Parameters
        self.m  = physics.mass
        com = physics.center_of_mass
        self.lf = math.sqrt(pow((com.x - physics.wheels[0].position.x), 2))#abs(physics.wheels[0].x)
        self.lr = math.sqrt(pow((com.x - physics.wheels[2].position.x), 2))
        self.Iz = physics.moi
        self.Df = 0.8 * self.m * 9.81 / 2.0
        self.Cf = 1.25
        self.Bf = 1.0
        self.Dr = 0.8 * self.m * 9.81 / 2.0
        self.Cr = 1.25
        self.Br = 1.0
        # self.m  = 1.98
        # self.lf = 0.125
        # self.lr = 0.125
        # self.Iz = 0.024
        # self.Df = 0.8 * m * 9.81 / 2.0
        # self.Cf = 1.25
        # self.Bf = 1.0
        # self.Dr = 0.8 * m * 9.81 / 2.0
        # self.Cr = 1.25
        # self.Br = 1.0
    def _DynModel(self, x, x_glob, u, np, dt, PointAndTangent):
        # This function computes the system evolution. Note that the discretization is deltaT and therefore is needed that
        # dt <= deltaT and ( dt / deltaT) = integer value
    
        
        
    
        # Discretization Parameters
        deltaT = 0.001
        x_next     = np.array([0., 0., 0., 0., 0., 0.], dtype=float)
        cur_x_next = np.array([0., 0., 0., 0., 0., 0.], dtype=float)
    
        # Extract the value of the states
        delta = u[0]
        a     = u[1]
    
        psi = x_glob[3]
        X = x_glob[4]
        Y = x_glob[5]
    
        vx    = x[0]
        vy    = x[1]
        wz    = x[2]
        epsi  = x[3]
        s     = x[4]
        ey    = x[5]
    
        # Initialize counter
        i = 0
        while( (i+1) * deltaT <= dt):
            # Compute tire slip angle
            alpha_f = delta - math.atan2( vy + self.lf * wz, vx )
            alpha_r = - math.atan2( vy - self.lr * wz , vx)
    
            # Compute lateral force at front and rear tire
            Fyf = self.Df * math.sin( self.Cf * math.atan(self.Bf * alpha_f ) )
            Fyr = self.Dr * math.sin( self.Cr * math.atan(self.Br * alpha_r ) )
    
            # Propagate the dynamics of deltaT
            x_next[0] = vx  + deltaT * (a - 1 / self.m * Fyf * math.sin(delta) + wz*vy)
            x_next[1] = vy  + deltaT * (1 / self.m * (Fyf * math.cos(delta) + Fyr) - wz * vx)
            x_next[2] = wz  + deltaT * (1 / self.Iz *(self.lf * Fyf * math.cos(delta) - self.lr * Fyr) )
            x_next[3] = psi + deltaT * (wz)
            x_next[4] =   X + deltaT * ((vx * math.cos(psi) - vy * math.sin(psi)))
            x_next[5] =   Y + deltaT * (vx * math.sin(psi)  + vy * math.cos(psi))
    
            cur = Curvature(s, PointAndTangent)
            cur_x_next[0] = x_next[0]
            cur_x_next[1] = x_next[1]
            cur_x_next[2] = x_next[2]
            cur_x_next[3] = epsi + deltaT * ( wz - (vx * math.cos(epsi) - vy * math.sin(epsi)) / (1 - cur * ey) * cur )
            cur_x_next[4] = s    + deltaT * ( (vx * math.cos(epsi) - vy * math.sin(epsi)) / (1 - cur * ey) )
            cur_x_next[5] = ey   + deltaT * (vx * math.sin(epsi) + vy * math.cos(epsi))
    
            # Update the value of the states
            psi  = x_next[3]
            X    = x_next[4]
            Y    = x_next[5]
    
            vx   = cur_x_next[0]
            vy   = cur_x_next[1]
            wz   = cur_x_next[2]
            epsi = cur_x_next[3]
            s    = cur_x_next[4]
            ey   = cur_x_next[5]
    
            # if (s < 0):
            #     print("Start Point: ", x, " Input: ", u)
            #     print("x_next: ", x_next)
    
            # Increment counter
            i = i+1
    
        # Noises
        noise_vx = np.max([-0.05, np.min([np.random.randn() * 0.01, 0.05])])
        noise_vy = np.max([-0.1, np.min([np.random.randn() * 0.01, 0.1])])
        noise_wz = np.max([-0.05, np.min([np.random.randn() * 0.005, 0.05])])
    
        cur_x_next[0] = cur_x_next[0] + 0.1*noise_vx
        cur_x_next[1] = cur_x_next[1] + 0.1*noise_vy
        cur_x_next[2] = cur_x_next[2] + 0.1*noise_wz
    
        return cur_x_next, x_next
    
    
    def wrap(angle):
        if angle < -np.pi:
            w_angle = 2 * np.pi + angle
        elif angle > np.pi:
            w_angle = angle - 2 * np.pi
        else:
            w_angle = angle
    
        return w_angle