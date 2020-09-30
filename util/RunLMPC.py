# ----------------------------------------------------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that you provide clear attribution to UC Berkeley,
# including a reference to the papers describing the control framework:
# [1] Ugo Rosolia and Francesco Borrelli. "Learning Model Predictive Control for Iterative Tasks. A Data-Driven
#     Control Framework." In IEEE Transactions on Automatic Control (2017).
#
# [2] Ugo Rosolia and Francesco Borrelli "Learning how to autonomously race a car: a predictive control approach" 
#     In 2017 IEEE Conference on Decision and Control (CDC)
#
# [3] Ugo Rosolia and Francesco Borrelli. "Learning Model Predictive Control for Iterative Tasks: A Computationally
#     IEEE Transactions on Control Systems Technology (2019).
#
# Attibution Information: Code developed by Ugo Rosolia
# (for clarifiactions and suggestions please write to ugo.rosolia@berkeley.edu).
#
# Code description: Simulation of the Learning Model Predictive Controller (LMPC). The main file runs:
# 1) A PID path following controller
# 2) A MPC which uses a LTI model identified from the data collected with the PID in 1)
# 3) A MPC which uses a LTV model identified from the date collected in 1)
# 4) A LMPC for racing where the safe set and value function approximation are build using the data from 1), 2) and 3)
# ----------------------------------------------------------------------------------------------------------------------


from SysModel import Simulator, PID, DynamicModel
from Classes import ClosedLoopData, LMPCprediction
from PathFollowingLTVMPC import PathFollowingLTV_MPC
from PathFollowingLTIMPC import PathFollowingLTI_MPC
from LMPC import ControllerLMPC
from Utilities import Regression
import numpy as np
import pdb
import pickle


class RaceLMPC(object):
    def __init__(self, dt, Time, maxlaps, map, physics):
        # ======================================================================================================================
        # ============================ Choose which controller to run ==========================================================
        # ======================================================================================================================
        # self.RunPID     = 1
        # self.RunMPC     = 1 
        # self.RunMPC_tv  = 1 
        # self.RunLMPC    = 1
        
        # ======================================================================================================================
        # ============================ Initialize parameters for path following ================================================
        # ======================================================================================================================
        self.dt         = dt       # Controller discretization time
        self.Time       = 200            # Simulation time for PID
        self.TimeMPC    = None             # Simulation time for path following MPC
        self.TimeMPC_tv = None            # Simulation time for path following LTV-MPC
        self.vt         = 40.0            # Reference velocity for path following controllers
        self.v0         = None         # Initial velocity at lap 0
        self.N          = 12              # Horizon length
        self.n = 6;   self.d = 2               # State and Input dimension
        self.points = int(self.Time/dt)
        self.count = 0
        # self.PIDinit = 0
        # self.LTIinit = 0
        # self.LTVinit = 0
        # self.LMPCinit = 0
        # Path Following tuning
        self.Q = np.diag([1.0, 1.0, 1, 1, 0.0, 100.0]) # vx, vy, wz, epsi, s, ey
        self.R = np.diag([1.0, 10.0])                  # delta, a
        #LTI-MPC Params
        self.A = None
        self.B = None
        self.error = None
        self.u = np.array([0., 0.], dtype=float)
        
        self.map = map                           # Initialize the map
        self.Dynamic = DynamicModel(physics)
        self.simulator = Simulator(self.Dynamic, self.map)                # Initialize the Simulator
        
        # ======================================================================================================================
        # ==================================== Initialize parameters for LMPC ==================================================
        # ======================================================================================================================
        
        # Safe Set Parameters
        self.LMPC_Solver = "CVX"           # Can pick CVX for cvxopt or OSQP. For OSQP uncomment line 14 in LMPC.py
        self.numSS_it = 4                  # Number of trajectories used at each iteration to build the safe set
        self.numSS_Points = 20             # Number of points to select from each trajectory to build the safe set
        
        self.Laps       = maxlaps + self.numSS_it      # Total LMPC laps
        self.TimeLMPC   = 400              # Simulation time
        
        # Tuning Parameters
        self.Qslack  = 20 * np.diag([10, 1, 1, 1, 10, 1])            # Cost on the slack variable for the terminal constraint
        self.Qlane   =  1 * np.array([0, 10])                        # Quadratic and linear slack lane cost
        self.Q_LMPC  =  0 * np.diag([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # State cost x = [vx, vy, wz, epsi, s, ey]
        self.R_LMPC  =  0 * np.diag([1.0, 1.0])                      # Input cost u = [delta, a]
        self.dR_LMPC = 10 * np.array([1.0, 10.0])                    # Input rate cost u
        
        self.inputConstr = np.array([[0.8, 0.8],                     # Min Steering and Max Steering
                                [10.0, 10.0]])                    # Min Acceleration and Max Acceleration        
    
    def PIDinit(self, x):
        self.v0 = x[0]
        self.ClosedLoopDataPID = ClosedLoopData(self.points , self.v0)
        self.PIDController = PID(self.vt)
        print("Starting PID")
        self.count = 0

    def PIDcont(self, x, x_glob, LapTime, SimTime, EndFlag=0):
    # ======================================================================================================================
    # ======================================= PID path following ===========================================================
    # ======================================================================================================================         
        if EndFlag == 0:
            
            u, new_x, new_x_glob = self.simulator.Sim(x, x_glob, SimTime, self.PIDController)
            self.ClosedLoopDataPID.UpdateLoop(self.count, x, x_glob, u, SimTime)
            self.count += 1
            return new_x, new_x_glob
        
        else:
            file_data = open('D:\CARLA\PythonAPI\examples\data\ClosedLoopDataPID.obj', 'wb')
            pickle.dump(self.ClosedLoopDataPID, file_data)
            file_data.close()
            print("===== PID terminated")
            self.PIDinit = 0 #It will run 2nd time
            
            print("Starting LTI-MPC")
            lamb = 0.0000001
            self.A, self.B, self.Error = Regression(self.ClosedLoopDataPID.x, self.ClosedLoopDataPID.u, lamb)
            return new_x, new_x_glob 
    
    def LTIinit(self, x, LapTime):
        self.TimeMPC = LapTime
        self.points = int(self.TimeMPC/self.dt)
        self.v0 = x[0]
        self.ClosedLoopDataLTI_MPC = ClosedLoopData(self.points, self.v0)
        self.Controller_PathFollowingLTI_MPC = PathFollowingLTI_MPC(self.A, self.B, self.Q, self.R, self.N, self.vt, self.inputConstr)
        # simulator.Sim(ClosedLoopDataLTI_MPC, Controller_PathFollowingLTI_MPC)
        self.count = 0

    def LTIcont(self, x, x_glob, LapTime, SimTime, EndFlag=0):
    # ======================================================================================================================
    # ======================================  LINEAR REGRESSION ============================================================
    # ======================================================================================================================
        if EndFlag == 0:
            u, new_x, new_x_glob = self.simulator.Sim(x, x_glob, SimTime, self.Controller_PathFollowingLTI_MPC)
            self.ClosedLoopDataLTI_MPC.UpdateLoop(self.count, x, x_glob, u, SimTime)
            
            self.count += 1
            return new_x, new_x_glob
        
        else:
            file_data = open('D:\CARLA\PythonAPI\examples\data\ClosedLoopDataLTI_MPC.obj', 'wb')
            pickle.dump(self.ClosedLoopDataLTI_MPC, file_data)
            file_data.close()
            
            print("===== MPC terminated")
            return new_x, new_x_glob
            
        
    
    def LTVinit(self, x, LapTime):
        print("Starting TV-MPC")
        file_data = open('D:\CARLA\PythonAPI\examples\data\ClosedLoopDataPID.obj', 'rb')
        ClosedLoopDataPID = pickle.load(file_data)
        self.TimeMPC_tv = LapTime
        self.points = int(self.TimeMPC_tv/self.dt)
        self.v0 = x[0]
        self.ClosedLoopDataLTV_MPC = ClosedLoopData(self.points, self.v0)
        self.Controller_PathFollowingLTV_MPC = PathFollowingLTV_MPC(self.Q, self.R, self.N, self.vt, self.n, self.d, ClosedLoopDataPID.x, ClosedLoopDataPID.u, self.dt, self.map, self.inputConstr)
        # simulator.Sim(ClosedLoopDataLTI_MPC, Controller_PathFollowingLTI_MPC)
        self.count = 0
        file_data.close()
        
    def LTVcont(self, x, x_glob, LapTime, SimTime, EndFlag=0):
    # ======================================================================================================================
    # ===================================  LOCAL LINEAR REGRESSION =========================================================
    # ======================================================================================================================      
        if EndFlag == 0:
            u, new_x, new_x_glob = self.simulator.Sim(x, x_glob, SimTime, self.Controller_PathFollowingLTV_MPC)
            self.ClosedLoopDataLTV_MPC.UpdateLoop(self.count, x, x_glob, u, SimTime)
            
            self.count += 1
            return new_x, new_x_glob
        else: 
            file_data = open('D:\CARLA\PythonAPI\examples\data\ClosedLoopDataLTV_MPC.obj', 'wb')
            pickle.dump(self.ClosedLoopDataLTV_MPC, file_data)
            file_data.close()
            
            print("===== TV-MPC terminated")
            return new_x, new_x_glob
            
    # ======================================================================================================================
    # ==============================  LMPC w\ LOCAL LINEAR REGRESSION ======================================================
    # ======================================================================================================================
    def LMPCinit(self, x, LapTime):
        print("Starting LMPC")
        self.TimeLMPC = LapTime
        self.points = int(self.TimeLMPC/self.dt)
        self.v0 = x[0]
        self.ClosedLoopLMPC = ClosedLoopData(self.points, self.v0)
        self.LMPCOpenLoopData = LMPCprediction(self.N, self.n, self.d, self.TimeLMPC, self.numSS_Points, self.Laps)
        self.LMPCSimulator = Simulator(self.Dynamic, map, 1, 1)
        self.LMPController = ControllerLMPC(self.numSS_Points, self.numSS_it, self.N, self.Qslack, self.Qlane, self.Q_LMPC,
                                       self.R_LMPC, self.dR_LMPC, self.dt, self.map, self.Laps, self.TimeLMPC, self.LMPC_Solver, self.inputConstr)
        
        PID_data = open('D:\CARLA\PythonAPI\examples\data\ClosedLoopDataPID.obj', 'rb')
        ClosedLoopDataPID = pickle.load(PID_data)
        LTV_data = open('D:\CARLA\PythonAPI\examples\data\ClosedLoopDataLTV_MPC.obj','rb')
        ClosedLoopDataLTV_MPC = pickle.load(LTV_data)
        LTI_data = open('D:\CARLA\PythonAPI\examples\data\ClosedLoopDataLTI_MPC.obj','rb')
        ClosedLoopDataLTI_MPC = pickle.load(LTI_data)
        
        # Run first 4 laps. Remaining laps run by LMPC
        self.LMPController.addTrajectory(ClosedLoopDataPID)
        self.LMPController.addTrajectory(ClosedLoopDataLTV_MPC)
        self.LMPController.addTrajectory(ClosedLoopDataPID)
        self.LMPController.addTrajectory(ClosedLoopDataLTI_MPC)
        
        PID_data.close()
        LTV_data.close()
        LTI_data.close()
        
        self.count = 0        
    
    def LMPCcont(self, x, x_glob, LapTime, SimTime, LapTrigger=0, EndFlag=0):       
        if EndFlag == 0:
            u, new_x, new_x_glob = self.LMPCSimulator.Sim(x, x_glob, SimTime, self.LMPController, self.LMPCOpenLoopData)
            self.ClosedLoopLMPC.UpdateLoop(self.count, x, x_glob, u, SimTime)
        
            self.count += 1
            
            if LapTrigger == 1:
                
                self.LMPController.addTrajectory(self.ClosedLoopLMPC)
                self.LMPCSimulator.reset()
                self.count = 0
            
            return new_x, new_x_glob
            
        else:
            file_data = open('D:\CARLA\PythonAPI\examples\data\LMPController.obj', 'wb')
            pickle.dump(self.ClosedLoopLMPC, file_data)
            pickle.dump(self.LMPController, file_data)
            pickle.dump(self.LMPCOpenLoopData, file_data)
            file_data.close()
            
            return new_x, new_x_glob
        
            print("===== LMPC terminated")
        
        
    # ======================================================================================================================
    # ========================================= PLOT TRACK =================================================================
    # ======================================================================================================================
    # for i in range(0, LMPController.it):
    #     print("Lap time at iteration ", i, " is ", LMPController.Qfun[0, i]*dt, "s")
    
    
    
    #xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
        #Dont need any of this
    #xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    
