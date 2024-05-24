import numpy as np
import math
import time

from scipy.optimize import minimize, Bounds
from config import *

class Drone:
    def __init__(self, state:np.array, control:np.array, radius):
        # Drone state and control
        self.time_stamp = 0.0
        self.state = state
        self.control = control

        self.n_state = 9
        self.n_control = 3

        # Drone radius
        self.radius = radius

        # Drone control bounds
        self.control_max = [ 1.0, 1.0, 1.0]
        self.control_min = [-1.0,-1.0,-1.0]
        
        # Store drone path
        self.path = [np.concatenate([[self.time_stamp], self.state, self.control])]

    def updateState(self, control:np.array, dt:float):
        """
        Computes the states of drone after applying control signals
        """
        
        # Update
        position = self.state[:3]
        velocity = self.state[3:6]
        accelerate = self.state[6:]

        next_position = position + velocity*dt
        next_velocity = velocity + (accelerate-D_FRAC*velocity)*dt
        next_accelerate = accelerate + control*dt

        self.state = np.concatenate([next_position, next_velocity, next_accelerate])
        self.control = control
        self.time_stamp = self.time_stamp + dt

        # Store
        self.path.append(np.concatenate([[self.time_stamp], self.state, self.control]))

    def setupController(self, horizon_length=20, dt=0.1):
        # Boundary
        self.upper_bound = self.control_max*horizon_length
        self.lower_bound = self.control_min*horizon_length

        # nmpc timestep
        self.nmpc_timestep = dt

        # Predictive length
        self.horizon_length = horizon_length

        # History predictive horizon
        self.prediction = np.zeros(self.state.shape[0]*horizon_length)
        self.predictions = [self.prediction]
    
        # The initial random control signals
        self.u0 = np.random.rand(self.control.shape[0]*self.horizon_length)

    def computeControlSignal(self):
        """
        Computes control velocity of the copter
        """
        state = self.state.copy()
        obstacles = self.observerObstacles()
        traj_ref = self.getOrientedGoalTrajectory()

        def cost_fn(u): return self.costFunction(u, state, traj_ref, obstacles)

        bounds = Bounds(self.lower_bound, self.upper_bound)

        u0 = self.u0
        res = minimize(cost_fn, u0, method='SLSQP', bounds=bounds, tol=10e-3)
        control = res.x[:3]
        self.prediction = self.predictTrajectory(state, res.x)
        self.u0 = res.x

        self.predictions.append(self.prediction)
        return control

    def getOrientedGoalTrajectory(self):
        dir = (GOAL-self.state[:3])/np.linalg.norm(GOAL-self.state[:3])
        # print(dir)
        path = [self.state[:3]]
        for i in range(1,self.horizon_length):
            # print("index {}: {}".format(i, np.linalg.norm(goal-path[-1])))
            if np.linalg.norm(GOAL-path[-1]) <= VREF*self.nmpc_timestep:
                path.append(GOAL)
            else:
                path.append(path[-1] + VREF*dir*self.nmpc_timestep)
        return np.array(path)

    def costFunction(self, u, state, traj_ref, obstacles):
        traj = self.predictTrajectory(state, u)

        c_u = self.costControl(u)
        c_tra = self.costTracking(traj, traj_ref)
        c_nav = self.costNavigation(traj)
        c_obs = self.costObstacle(traj, obstacles)
        total = W_tra*c_tra + W_nav*c_nav + W_u*c_u + W_obs*c_obs

        return total

    def costControl(self, u):
        return np.sum(u**2)

    def costTracking(self, traj, traj_ref):
        cost_tra = 0
        for i in range(self.horizon_length):
            pos_rel = traj[self.n_state*i:self.n_state*i+3] - traj_ref[i,:]
            cost_tra += np.sum(pos_rel**2)
        return cost_tra
    
    def costNavigation(self, traj):
        cost_nav = 0
        for i in range(self.horizon_length):
            vel = traj[self.n_state*i+3:self.n_state*i+6]
            cost_nav += (np.sum(vel**2) - VREF**2)**2
        return cost_nav
    
    def costObstacle(self, traj, obstacles):
        cost_obs = 0
        for j in range(obstacles.shape[0]):
            obs = obstacles[j,:]
            for i in range(self.horizon_length):
                obs_rel = traj[self.n_state*i:self.n_state*i+2] - obs[:2]
                cost_obs += 1 / (1 + np.exp(20*(np.linalg.norm(obs_rel) - ROBOT_RADIUS - obs[2])))
        return cost_obs
    
    def observerObstacles(self):
        observed_obstacles = []
        for i in range(OBSTACLES.shape[0]):
            if np.hypot(self.state[0]-OBSTACLES[i,0],
                        self.state[1]-OBSTACLES[i,1]) < SENSING_RADIUS + OBSTACLES[i,2]:
                observed_obstacles.append(OBSTACLES[i,:])
        return np.array(observed_obstacles)

    def predictTrajectory(self, state, controls):
        """
        Computes the states of the system after applying a sequence of control signals u on
        initial state x0
        """
        trajectory = []
        for i in range(self.horizon_length):
            # Update
            position = state[:3]
            velocity = state[3:6]
            accelerate = state[6:]
            control = controls[self.n_control*i:self.n_control*(i+1)]

            next_position = position + velocity*self.nmpc_timestep
            next_velocity = velocity + (accelerate-D_FRAC*velocity)*self.nmpc_timestep 
            next_accelerate = accelerate + control*self.nmpc_timestep
            state = np.concatenate([next_position, next_velocity, next_accelerate])
            trajectory.append(state)
        return np.array(trajectory).reshape((state.shape[0]*self.horizon_length))
    

