#!/usr/bin/env python3
import numpy as np
import casadi as ca
import scipy.linalg
import cvxpy as cp
import sympy as sp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import rospy as rp
import os
import json
class Container:
    def __init__(self, beta, lambda_forget, sigma_forget, horizon_size=40):
        self.horizon_size = horizon_size

        self.dp = np.zeros((horizon_size,3))

        self.disturbance = np.zeros((horizon_size,3))
        self.rho = np.zeros((horizon_size,3))
        self.delta_s = np.zeros(horizon_size)
        self.errors = np.zeros((horizon_size,3))
        self.actuations = np.zeros((horizon_size,3))
        self.current_index = 0
        self.is_filling = True
        self.beta = beta
        self.lambda_forget = lambda_forget
        self.sigma_forget = sigma_forget
        self.previous_jacobians = []
        
        


    def random_init(self):
        
        self.dp = np.random.rand(self.horizon_size,3)
      
        self.disturbance = np.random.rand(self.horizon_size,3)
        self.rho = np.random.rand(self.horizon_size,3)
        self.delta_s = np.random.rand(self.horizon_size)
        self.errors = np.random.rand(self.horizon_size,3)


    def get_tip_pose_in_base(base, tip):
        tip_pose_base = np.linalg.inv(base) @ tip
        actual_tip_pos = tip_pose_base[0:3, 3]
        
        
    def detect_dead_zone_EMA(
            self,
            actuations,
            starting_point,
            ending_point,
            alpha=0.2,
            threshold=0.15
        ):
        """
        Detect whether the tip has moved sufficiently between two positions.
        Applies EMA filtering to reduce sensitivity to micro-jitter.

        Parameters
        ----------
        actuations : list or iterable
            Actuation command values for this step. If all 0 → movement not intentional.
        starting_point : np.array
            Tip position before actuation.
        ending_point : np.array
            Tip position after actuation.
        alpha : float
            EMA smoothing factor.
        threshold : float
            Minimum movement required to consider the step "valid".

        Returns
        -------
        int
            1 if valid movement detected, 0 if filtered out as dead zone.
        """

        # If the actuation command is zero → no movement intended
        if all(a == 0 for a in actuations):
            return 0

        # Initialize EMA memories if missing
        if not hasattr(self, 'last_filtered_start'):
            self.last_filtered_start = starting_point
        if not hasattr(self, 'last_filtered_end'):
            self.last_filtered_end = ending_point

        # --- EMA filter ---
        starting_filtered = alpha * starting_point + (1 - alpha) * self.last_filtered_start
        ending_filtered   = alpha * ending_point   + (1 - alpha) * self.last_filtered_end

        # Update stored filtered states
        self.last_filtered_start = starting_filtered
        self.last_filtered_end   = ending_filtered

        # --- Compute norm ---
        norm = np.linalg.norm(ending_filtered - starting_filtered)

        # Dead-zone detection
        if norm < threshold:
            print("NORM: ",norm)
            return 0
        else:
            rp.loginfo("Valid Movement Detected")
            return 1

        
        
        
        
        
        
        
    def detect_dead_zone(self, actuations, starting_point, ending_point, threshold=0.6):
        """
        Detects if the tip has actually moved beyond a threshold during an actuation step.
        Filtering with a simple EMA reduces sensitivity to micro-jitter.
        """
        if all(a == 0 for a in actuations):
            return 0



        # --- Norm filtrata ---
        norm = np.linalg.norm(starting_point - ending_point)



        if norm < threshold:
            print("NORM: ",norm)
            rp.loginfo("Hysteresis / deadzone detected!")
            return 0
        else:
            rp.loginfo("Valid movement detected!")
            print("NORM: ",norm)            
            return 1
    

    def update_observations(self, actuations, p_meas, p_target, movement_norm):

        if self.current_index == self.horizon_size:
            self.current_index = 0
            if self.is_filling:
                self.is_filling = False
        self.dp[self.current_index] = p_target - p_meas
        self.detect_dead_zone(actuations, p_meas, p_target)
        self.delta_s[self.current_index] = movement_norm
        actuations[0] = actuations[0] 
        actuations[1] = actuations[1] 
        actuations[2] = actuations[2]
        self.actuations[self.current_index] = actuations
        self.current_index += 1
    
           
        
    def update_observations_init(self, actuations, p_meas, p_target, movement_norm):
        if self.detect_dead_zone(actuations, p_meas, p_target):
            if self.current_index == self.horizon_size:
                self.current_index = 0
                if self.is_filling:
                    self.is_filling = False
            self.dp[self.current_index] = p_target - p_meas
            
            self.delta_s[self.current_index] = movement_norm
            actuations[0] = actuations[0] 
            actuations[1] = actuations[1] 
            actuations[2] = actuations[2] 
            self.actuations[self.current_index] = actuations
            self.current_index += 1
            #print(self.current_index)
            #print(self.horizon_size)
        
    def get_state(self):
        return {
            "horizon_size": self.horizon_size,
            "current_index": self.current_index,
            "is_filling": self.is_filling,
            "beta": self.beta,
            "lambda_forget": self.lambda_forget,
            "sigma_forget": self.sigma_forget,
            "dp": self.dp.tolist(),
            "disturbance": self.disturbance.tolist(),
            "rho": self.rho.tolist(),
            "delta_s": self.delta_s.tolist(),
            "errors": self.errors.tolist(),
            "actuations": self.actuations.tolist(),
        }
    
    
    
    
    def save_state(self, directory="./logs", filename="container_state.json"):
        """Save the container state as a JSON file."""
        # Ensure the directory exists
        os.makedirs(directory, exist_ok=True)

        # Build full file path
        filepath = os.path.join(directory, filename)

        # Save JSON file
        with open(filepath, "w") as f:
            json.dump(self.get_state(), f, indent=2)

        print(f"✅ Container state saved to: {filepath}")
