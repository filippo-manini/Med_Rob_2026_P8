#!/usr/bin/env python3
import rospy as rp
import numpy as np
import matplotlib.pyplot as plt
import time 
import sympy as sp # For Jacobian matrix
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from scipy.signal import butter, filtfilt, find_peaks, argrelextrema
from nav_msgs.msg import Path
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
import math
import rospy as rp
from scipy.optimize import minimize
import os
import csv
import traceback
import sys
from controllers.controller_utilities import *
import pandas as pd
import yaml
def raise_with_line_info(e):
    exc_type, exc_value, exc_tb = sys.exc_info()
    tb_lines = traceback.format_exception(exc_type, exc_value, exc_tb)
    tb_text = "".join(tb_lines)
    # Re-raise the exception with full traceback info
    raise RuntimeError(f"Error at line {exc_tb.tb_lineno}:\n{tb_text}") from e


class ConstantCurvatureFitter:
    def __init__(self):
        pass  
    def cc_forward_kinematics_correct(self, k, theta, phi, l):
       
        x = (1 / k) * np.cos(phi) * (1 - np.cos(theta))
        y = (1 / k) * np.sin(phi) * (1 - np.cos(theta))
        z = (1 / k) * np.sin(theta)
        return np.array([x, y, z])

    def generate_curve_endpoint(self, tip_pose, k, theta, phi, l):
        """
        Compute the final position of the CC curve expressed wrt the base frame.
        """
        R_tip = tip_pose[:3, :3]
        t_tip = tip_pose[:3, 3]

        local_endpoint = self.cc_forward_kinematics_correct(k, theta, phi, l)
        global_endpoint = R_tip @ local_endpoint + t_tip
        return global_endpoint

    def fit_cc_params(self, tip_pose, target, initial_guess=None):
        if initial_guess is None:
            initial_guess = np.array([0.05, 1.0, 0.0, 40.0])
        
        bounds = [
            (1e-6, 0.5),        # curvature
            (0.0, 2 * np.pi),   # angle from 0 to 360°
            (-np.pi, np.pi),    # complete phi 
            (1.0, 200.0)        # maximum length 200 mm
        ]

        def constraint_eq(params):
            k, theta, phi, l = params
            return self.generate_curve_endpoint(tip_pose, k, theta, phi, l) - target

        constraint = {'type': 'eq', 'fun': constraint_eq}

        def objective(params):
            k, theta, phi, l = params
            return k**2 + 0.001 * l  # penalize k and l

        res = minimize(objective, initial_guess, bounds=bounds, constraints=[constraint], method='SLSQP')

        if res.success or "Positive directional derivative" in res.message:
            k_opt, theta_opt, phi_opt, l_opt = res.x
            end_error = np.linalg.norm(self.generate_curve_endpoint(tip_pose, k_opt, theta_opt, phi_opt, l_opt) - target)
            #print(f"Success! Endpoint error = {end_error:.6e}")
            return {
                'k': k_opt,
                'theta': theta_opt,
                'phi': phi_opt,
                'l': l_opt,
                'fun': res.fun
            }
        else:
            raise RuntimeError("Optimization Failed: " + res.message)



    
        
class Controller:
    def __init__(self, 
                 container: 'Container',
                 jacobian_estimator: 'JacobianEstimator',
                 trajectory_estimator: 'TrajectoryEstimator',
                 kalman_predictor: 'KalmanPredictor', 
                 pid_controller: 'PID_Controller',
                 iteration,
                 delta_t_btw_actuations,
                 dl_to_steps):
        self.container = container
        self.jacobian_estimator = jacobian_estimator
        self.trajectory_estimator = trajectory_estimator
        self.kalman_predictor = kalman_predictor
        self.pid_controller = pid_controller
        self.Number_of_iteration = iteration
        self.delta_t_btw_actuations = delta_t_btw_actuations
        self.dl_to_steps = np.asarray(dl_to_steps)
        self.processor = None  # e.g., Aurora data handler
        self.communicator = None  # e.g., Arduino handler
        self.plotter = None
        self.analytical = False
        self.motor_threshold= 1500        
        self.dynamical_weight_update = False    
        self.estimate_jacobian = True
        self.max_ml_per_step = 20
        self.max_ap_per_step = 20
        self.max_ins_per_step = 150
        self.max_limits = [
            self.max_ins_per_step,
            self.max_ml_per_step,
            self.max_ap_per_step    
        ]
        self.stopping_threshold = 0.5
        self.workspace_mapper = WorkspaceMapper()
        self.trajectory_estimator.workspace_mapper = self.workspace_mapper
        

        self.path_pub = rp.Publisher('/trajectory_path', Path, queue_size=10)
        self.waypoint_pub = rp.Publisher('/waypoint', PointStamped, queue_size=10)
        self.points_pub = rp.Publisher('/trajectory_points', Marker, queue_size=10)
        
        
    


        
        
    def calculate_tf_for_valve(self):
        history = []
        for i in range(100):
            rp.sleep(0.025)
            if self.processor.valve_history:
                history.append(self.processor.valve_pose[:3, 3])

        valve_base = np.mean(history, axis=0)
        print(valve_base)
        print(self.processor.valve_base)

        self.processor.valve_T[:3, 3] = self.processor.valve_base - valve_base

        print(f"Transformation matrix found at {self.processor.valve_T}")
        



 
        
    def estimate_reference_from_minima(self,trajectory_3d):
        """
        Estimate the reference point (x0) along a motion direction
        using the mean of local minima projected onto the direction vector.

        Parameters:
        - trajectory_3d: np.ndarray of shape (N, 3)
        - direction_unit_vector: np.ndarray of shape (3,)

        Returns:
        - x0: np.ndarray of shape (3,)
        """
        # Project all points onto the direction vector
        scalar_projections = np.dot(trajectory_3d - trajectory_3d[0], self.valve_motion_direction)

        # Find local minima indices (you can tune `order` based on your noise level)
   
        local_min_indices = argrelextrema(scalar_projections, np.less, order=3)[0]
            
        if len(local_min_indices) == 0:
            raise ValueError("No local minima found. Check input trajectory or noise level.")

        # Get corresponding 3D points
        minima_points = trajectory_3d[local_min_indices]

        # Compute mean
        x0_estimated = np.mean(minima_points, axis=0)
        return x0_estimated
    

    def estimate_frequency_peak_method(self, trajectory_3d, sampling_rate=10.0, motion_direction=None, min_peak_distance_sec=0.2):
            """
            Estimate frequency using a Pan-Tompkins-like peak detection method.

            Parameters:
            - trajectory_3d: (N, 3) array
            - sampling_rate: sampling frequency (Hz)
            - motion_direction: unit vector (optional)

            Returns:
            - estimated_freq: in Hz
            """
            # 1. Project to scalar


            scalar_signal = np.dot(trajectory_3d - self.valve_base, self.valve_motion_direction)
            '''
            plt.figure(figsize=(10, 4))  
            plt.plot(scalar_signal, label='Signal')

            plt.xlabel("Time (s)")
            plt.ylabel("Amplitude")
            plt.title("Signal overtime")
            plt.grid(True)
            plt.legend()
            plt.tight_layout()
            plt.show()
            '''

            # 2. Smooth the signal
            fs = 10  # sampling frequency(Hz)
            cutoff = 3 # Cutoff frequency (Hz)
            order = 2

            b, a = butter(order, cutoff / (0.5 * fs), btype='low')
            scalar_signal = filtfilt(b, a, scalar_signal)

            # 3. Detect peaks (Pan-Tompkins inspired)
            min_distance = int(0.1 * sampling_rate)
            peaks, _ = find_peaks(scalar_signal)

            # 4. Compute inter-peak intervals
            if len(peaks) < 2:
                print("Not enough peaks found.")
                return 0.0

            peak_intervals = np.diff(peaks) / sampling_rate
            avg_period = np.mean(peak_intervals)
            estimated_freq = 1.0 / avg_period

            return estimated_freq / 2  , scalar_signal
        

        
        
    def identify_model(self):
        # Collect samples to derive motion direction and baseline
        history = []
        start = time.time()
        for i in range(200):
            rp.sleep(0.1)  # wait for the new datat to come
            if self.processor.valve_history:
                history.append(self.processor.valve_pose[:3,3])  # get latest valve pose
                
        end = time.time()
        
        

        print(f"Tempo impiegato: {end - start:.4f} secondi")
        print("data collected, ready for model identification")
        start = time.time()

        history = np.array(history)  # convert to numpy array for analysis

        if history.shape[0] < 2:
            print("Not enough data collected for model identification.")
            return

        # Estimate a direction vector using PCA (SVD)
        centered = history - np.mean(history, axis=0)
        u, s, vh = np.linalg.svd(centered, full_matrices=False)
        motion_direction = vh[0]  # principal motion direction

        print(f"Identified motion direction: {motion_direction}")
        self.valve_motion_direction = motion_direction
        
        x0 = self.estimate_reference_from_minima(history)
        print(f"coordinates of the valve base found at: {x0}")
        self.valve_base = x0
        
        frequency, scalar_signal = self.estimate_frequency_peak_method(history)
        print(f"frequency estimated: {frequency}")
        self.kalman_predictor.f = frequency
        end = time.time()
        
        

        print(f"Tempo impiegato: {end - start:.4f} secondi")
            

                    
        ending_index = int( 5 / self.kalman_predictor.f)     
        last_sinusoid = scalar_signal[-ending_index:]
        max_index = np.argmax(last_sinusoid)
        

        wait = max_index + 1 + int(len(last_sinusoid)/2) 
        self.processor.model_identified = True
        return wait


    def initialize_jacobian(self, delta_ml = 20, delta_ap = 15, delta_ins = 100, number_of_valid_samples_ap_ml = 10, number_of_valid_samples_ins = 30):
        self.container.__init__( self.container.beta, self.container.lambda_forget, self.container.sigma_forget, int( 4 * number_of_valid_samples_ap_ml + number_of_valid_samples_ins))

        self.processor.collect_data_for_jacobian_ins(delta_ins, number_of_valid_samples_ins)
        #self.processor.collect_data_for_jacobian_ins(-delta_ins, number_of_valid_samples + 15) # use only insertion because of the 
        # huge position Hysteresis when inserting and then retracting the catheter

        self.processor.collect_data_for_jacobian_ml(-delta_ml, number_of_valid_samples_ap_ml)
        self.processor.collect_data_for_jacobian_ml(delta_ml, number_of_valid_samples_ap_ml)
        self.processor.collect_data_for_jacobian_ap(-delta_ap, number_of_valid_samples_ap_ml)
        self.processor.collect_data_for_jacobian_ap(delta_ap, number_of_valid_samples_ap_ml)
        
       
        self.jacobian_estimator.initialize_jacobian()

    
                
    def estimate_jacobian_if_necessary(self):
        if self.estimate_jacobian and not self.use_previously_estimated_jacobian:  
             
            
            # initialize the Jacobian
            
            horizon = self.container.horizon_size
            self.initialize_jacobian(number_of_valid_samples_ap_ml = 15, delta_ins=150, number_of_valid_samples_ins = 30)
            self.container.__init__( self.container.beta, self.container.lambda_forget, self.container.sigma_forget, horizon_size=horizon)
            self.container.is_filling = True             
            path = os.path.join(os.path.dirname(__file__), '..', '..', 'config')
            print('calibration file saved at: ', path)
            if not os.path.exists(path):
                os.makedirs(path)
                
            with open(os.path.join(path, 'jacobian_initialization.txt'), 'w') as f:
                f.write(f'{self.jacobian_estimator.actual_jac}\n')
                
                
                
            self.set_controller_use_previously_estimated_jacobian(True)
        else:
            file_path = os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'jacobian_initialization.txt')
            print(f"attempt to open the file: {os.path.dirname(os.path.abspath(file_path))}")
            if (os.path.exists(file_path)):
                with open(file_path, 'r') as f:
                    lines = f.readlines()

                    # Process T_tip_calibration
                    actual_jac = []
                    for line in lines:
                        line = line.strip()
                        line = line.replace('[', '').replace(']', '')
                        elements = [float(element) for element in line.split()]
                        actual_jac.append(elements)
                    self.jacobian_estimator.actual_jac = np.array(actual_jac)
                    
                    print("JACOBIAN SUCCESSFULLY LOADED AT: ",self.jacobian_estimator.actual_jac)           
     
     
    def collect_data_for_init(self):
        self.actuate_init(
            np.array((5000, 0, 0)),
            self.processor.valve_pose[:3, 3].copy(),
            self.processor.valve_pose[:3, 3].copy()
        )
        self.actuate_init(
            np.array((-5000, 0, 0)),
            self.processor.valve_pose[:3, 3].copy(),
            self.processor.valve_pose[:3, 3].copy()
        )

           
     
                
    def generate_waypoint(self):
        actual_tip_position = self.processor.tip_pose.copy()
        valve_position = self.processor.valve_pose[:3, 3].copy()
        R = actual_tip_position[:3,:3]
        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt
        actual_tip_position = np.block([
            [R, actual_tip_position[:3,3:4]],
            [0, 0, 0, 1]])
        target = self.trajectory_estimator.express_point_in_frame(actual_tip_position, valve_position)
        #print("Final target position in tip frame:", target)
        #print("target reframed: ", actual_tip_position[:3, :3] @ target + actual_tip_position[:3, 3])
        l_guess, phi_guess, k_guess, theta_guess, f_k_target = self.jacobian_estimator.compute_CC_model(target)
        #print(f"parameters before optimization: k={k_guess:.6f}, theta={theta_guess:.6f} rad ({np.degrees(theta_guess):.2f}°), phi={phi_guess:.6f} rad ({np.degrees(phi_guess):.2f}°), l={l_guess:.6f}")

        # Generate initial guess trajectory based on CC model
        points = []
        fitter = ConstantCurvatureFitter()
        result = fitter.fit_cc_params(np.eye(4), target, initial_guess = np.array([k_guess, theta_guess, phi_guess, l_guess]))
        
        #print("after optimization",result)
        
        points = self.jacobian_estimator.generate_CC_points(actual_tip_position, result['k'], result["theta"], result['phi'], result['l'], 20)
        
        
        self.waypoint = points[7]
        
        print(f"WAYPOINT FOUND AT: {np.array2string(self.waypoint, precision=6)}, check Rviz for the visual hint" )

        self.publish_waypoint(self.waypoint, "world")
        
        self.publish_trajectory_points(points)
        
    def check_for_actuation_bounds(self, dl):
        if np.any(np.abs(dl) >  self.motor_threshold):
            rp.logerr(f"dl value exceeds limitations given in the .yaml file: {dl}. MUST BE ≤ ±{ self.motor_threshold}.")
            # finalize immediately on saturation
            if getattr(self, "plotter", None):
                self.plotter.stop_requested = True
                try:
                    self.plotter.finalize_one_shot(self.trajectory_estimator.workspace_mapper)   # plots + rewind + ROS shutdown
                finally:
                    return                   # leave Compute_MPC_Control cleanly
            else:
                rp.signal_shutdown("dl saturation without plotter attached")
                return
    
    
    



     
        
    def actuate_in_smaller_steps(self, actuations, target, valve_pos):
        # --- Compute number of steps safely ---
        steps = max(math.ceil(abs(v) / m) for v, m in zip(actuations, self.max_limits))
        elapsed_time = time.time() - self.processor.start_time

        norm_of_the_displacement = np.linalg.norm(self.processor.tip_pose[:3, 3] - target)
        if norm_of_the_displacement == 0:
            norm_of_the_displacement = 1e-9  # avoid division by zero
        print("Next actuation step BEFORE PRUNING:", actuations)
        actuations = self.workspace_mapper.prune_actuations(actuations)
        print("Next actuation step AFTER PRUNING:", actuations)
        # --- Prepare CSV row ---
        data_row = {
            "time": elapsed_time,
            "timestamp": time.time(),
            "base_pos_x": self.processor.base_pose[0, 3],
            "base_pos_y": self.processor.base_pose[1, 3],
            "base_pos_z": self.processor.base_pose[2, 3],
            "tip_pos_x": self.processor.tip_pose[0, 3],
            "tip_pos_y": self.processor.tip_pose[1, 3],
            "tip_pos_z": self.processor.tip_pose[2, 3],
            "valve_pos_x": valve_pos[0],
            "valve_pos_y": valve_pos[1],
            "valve_pos_z": valve_pos[2],
            "target_tip_pos_x": target[0],
            "target_tip_pos_y": target[1],
            "target_tip_pos_z": target[2],
            "ACTUATION_INS": actuations[0],
            "ACTUATION_ML": actuations[1],
            "ACTUATION_AP": actuations[2],
            # placeholders for results
            "reached_tip_pos_x": 0,
            "reached_tip_pos_y": 0,
            "reached_tip_pos_z": 0,
            "normalized_position_error": 0,
            "position_error": 0,
            "target_error": 0,
        }

        # --- Stepwise actuation ---
        remaining = actuations.copy()
        print("STEPS:", steps)

        for i in range(steps):
            scaled_actuations = [np.sign(r) * min(abs(r), m) for r, m in zip(remaining, self.max_limits)]
            print(f"STEP {i+1}/{steps} TO ACTUATE:", scaled_actuations)

            remaining = [r - s for r, s in zip(remaining, scaled_actuations)]

            if not np.allclose(scaled_actuations, 0):
                starting_position = self.processor.tip_pose[0:3, 3].copy()
                
                self.communicator.send_steps_to_arduino_dead_zone(delta_steps=scaled_actuations)
                self.workspace_mapper.add_insertion(scaled_actuations[0])
                self.workspace_mapper.add_step_AP(scaled_actuations[2])
                self.workspace_mapper.add_step_ML(scaled_actuations[1])
                
                self.workspace_mapper.get_catheter_state()
                moved = self.container.detect_dead_zone_EMA(
                    scaled_actuations, starting_position, self.processor.tip_pose[0:3, 3]
                )

                if moved:
                    self.container.update_observations(
                        scaled_actuations, starting_position, self.processor.tip_pose[0:3, 3], 0
                    )

            if np.allclose(remaining, 0):
                break  # stop if fully done

        # --- After actuation, log results ---
        data_row.update({
            "reached_tip_pos_x": self.processor.tip_pose[0, 3],
            "reached_tip_pos_y": self.processor.tip_pose[1, 3],
            "reached_tip_pos_z": self.processor.tip_pose[2, 3],
            "position_error": np.linalg.norm(self.processor.tip_pose[:3, 3] - target),
            "target_error": self.processor.target_error,
            "normalized_position_error": np.linalg.norm(
                (self.processor.tip_pose[:3, 3] - target) / norm_of_the_displacement
            ),
            "min_target_error": self.processor.min_target_error
        })

        with open(self.processor.run_paths["results_csv"], "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.processor.run_paths["fieldnames_dict"])
            writer.writerow(data_row)

        if self.estimate_jacobian:
            if not (self.container.is_filling and self.container.current_index == 0):
                self.jacobian_estimator.estimate_jacobian_dead_zone()


    def actuate_in_smaller_steps_ins_first(self, actuations, target, valve_pos):
        # --- Compute number of steps safely ---
        
        elapsed_time = time.time() - self.processor.start_time

        norm_of_the_displacement = np.linalg.norm(self.processor.tip_pose[:3, 3] - target)
        if norm_of_the_displacement == 0:
            norm_of_the_displacement = 1e-9  # avoid division by zero
        print("Next actuation step BEFORE PRUNING:", actuations)
        actuations = self.workspace_mapper.prune_actuations(actuations)
        print("Next actuation step AFTER PRUNING:", actuations)
        steps_per_axis = [math.ceil(abs(v) / m) for v, m in zip(actuations, self.max_limits)]
        steps = max(steps_per_axis)
        # --- Prepare CSV row ---
        data_row = {
            "time": elapsed_time,
            "timestamp": time.time(),
            "base_pos_x": self.processor.base_pose[0, 3],
            "base_pos_y": self.processor.base_pose[1, 3],
            "base_pos_z": self.processor.base_pose[2, 3],
            "tip_pos_x": self.processor.tip_pose[0, 3],
            "tip_pos_y": self.processor.tip_pose[1, 3],
            "tip_pos_z": self.processor.tip_pose[2, 3],
            "valve_pos_x": valve_pos[0],
            "valve_pos_y": valve_pos[1],
            "valve_pos_z": valve_pos[2],
            "target_tip_pos_x": target[0],
            "target_tip_pos_y": target[1],
            "target_tip_pos_z": target[2],
            "ACTUATION_INS": actuations[0],
            "ACTUATION_ML": actuations[1],
            "ACTUATION_AP": actuations[2],
            # placeholders for results
            "reached_tip_pos_x": 0,
            "reached_tip_pos_y": 0,
            "reached_tip_pos_z": 0,
            "normalized_position_error": 0,
            "position_error": 0,
            "target_error": 0,
        }

        # --- Stepwise actuation ---
        remaining = actuations.copy()
        print("STEPS:", steps)
        print("steps_per_axis", steps_per_axis)
        for i in range(steps):
            scaled_actuations = [np.sign(r) * min(abs(r), m) for r, m in zip(remaining, self.max_limits)]
            if i < steps - steps_per_axis[1]:
                scaled_actuations[1] = 0
            if i < steps - steps_per_axis[2]:
                scaled_actuations[2] = 0 
            print(f"STEP {i+1}/{steps} TO ACTUATE:", scaled_actuations)

            remaining = [r - s for r, s in zip(remaining, scaled_actuations)]

            if not np.allclose(scaled_actuations, 0):
                starting_position = self.processor.tip_pose[0:3, 3].copy()
                
                self.communicator.send_steps_to_arduino_dead_zone(delta_steps=scaled_actuations)
                self.workspace_mapper.add_insertion(scaled_actuations[0])
                self.workspace_mapper.add_step_AP(scaled_actuations[2])
                self.workspace_mapper.add_step_ML(scaled_actuations[1])
                
                self.workspace_mapper.get_catheter_state()
                moved = self.container.detect_dead_zone(
                    scaled_actuations, starting_position, self.processor.tip_pose[0:3, 3]
                )

                if moved:
                    self.container.update_observations(
                        scaled_actuations, starting_position, self.processor.tip_pose[0:3, 3], 0
                    )

            if np.allclose(remaining, 0):
                break  # stop if fully done

        # --- After actuation, log results ---
        data_row.update({
            "reached_tip_pos_x": self.processor.tip_pose[0, 3],
            "reached_tip_pos_y": self.processor.tip_pose[1, 3],
            "reached_tip_pos_z": self.processor.tip_pose[2, 3],
            "position_error": np.linalg.norm(self.processor.tip_pose[:3, 3] - target),
            "target_error": self.processor.target_error,
            "normalized_position_error": np.linalg.norm(
                (self.processor.tip_pose[:3, 3] - target) / norm_of_the_displacement
            ),
            "min_target_error": self.processor.min_target_error
        })

        with open(self.processor.run_paths["results_csv"], "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.processor.run_paths["fieldnames_dict"])
            writer.writerow(data_row)

        if self.estimate_jacobian:
            if not (self.container.is_filling and self.container.current_index == 0):
                self.jacobian_estimator.estimate_jacobian_lyapunof()



    def actuate_in_smaller_steps_ins_first_hysteresis(self, actuations, target, valve_pos):
            # --- Compute number of steps safely ---
            
            elapsed_time = time.time() - self.processor.start_time

            norm_of_the_displacement = np.linalg.norm(self.processor.tip_pose[:3, 3] - target)
            if norm_of_the_displacement == 0:
                norm_of_the_displacement = 1e-9  # avoid division by zero
            print("Next actuation step BEFORE PRUNING:", actuations)
            actuations = self.workspace_mapper.prune_actuations(actuations)
            print("Next actuation step AFTER PRUNING:", actuations)
            steps_per_axis = [math.ceil(abs(v) / m) for v, m in zip(actuations, self.max_limits)]
            steps = max(steps_per_axis)
            # --- Prepare CSV row ---
            data_row = {
                "time": elapsed_time,
                "timestamp": time.time(),
                "base_pos_x": self.processor.base_pose[0, 3],
                "base_pos_y": self.processor.base_pose[1, 3],
                "base_pos_z": self.processor.base_pose[2, 3],
                "tip_pos_x": self.processor.tip_pose[0, 3],
                "tip_pos_y": self.processor.tip_pose[1, 3],
                "tip_pos_z": self.processor.tip_pose[2, 3],
                "valve_pos_x": valve_pos[0],
                "valve_pos_y": valve_pos[1],
                "valve_pos_z": valve_pos[2],
                "target_tip_pos_x": target[0],
                "target_tip_pos_y": target[1],
                "target_tip_pos_z": target[2],
                "ACTUATION_INS": actuations[0],
                "ACTUATION_ML": actuations[1],
                "ACTUATION_AP": actuations[2],
                # placeholders for results
                "reached_tip_pos_x": 0,
                "reached_tip_pos_y": 0,
                "reached_tip_pos_z": 0,
                "normalized_position_error": 0,
                "position_error": 0,
                "target_error": 0,
            }

            # --- Stepwise actuation ---
            remaining = actuations.copy()
            print("STEPS:", steps)
            print("steps_per_axis", steps_per_axis)
            for i in range(steps):
                scaled_actuations = [np.sign(r) * min(abs(r), m) for r, m in zip(remaining, self.max_limits)]
                if i < steps - steps_per_axis[1]:
                    scaled_actuations[1] = 0
                if i < steps - steps_per_axis[2]:
                    scaled_actuations[2] = 0 
                print(f"STEP {i+1}/{steps} TO ACTUATE:", scaled_actuations)

                remaining = [r - s for r, s in zip(remaining, scaled_actuations)]

                if not np.allclose(scaled_actuations, 0):
                    
                    
                    
                    
                    starting_position = self.processor.tip_pose[0:3, 3].copy()
                    
                    if scaled_actuations[2] == self.max_ap_per_step or scaled_actuations[2] == -self.max_ap_per_step : 
                        
                        # only for full range
                        
                        # problem whenever we have posterior actuation and then ML or ins, the tendon might relax so we check for deadzone first
                        
                        # if deadzone is found we go out and then we set the starting condition to 800
                        ap = [0, 0, scaled_actuations[2]]
                        rest = [scaled_actuations[0], scaled_actuations[1], 0]
                        self.communicator.send_steps_to_arduino_dead_zone(delta_steps=ap)
                        
                        print("AP check")
                        
                        moved_ap = self.container.detect_dead_zone(
                            scaled_actuations, starting_position, self.processor.tip_pose[0:3, 3], threshold = 0.5
                        )

                        if not moved_ap:
                            
                            #move again to avoid false positive
                            self.communicator.send_steps_to_arduino_dead_zone(delta_steps=ap)
                        
                        
                            moved_ap = self.container.detect_dead_zone(
                                scaled_actuations, starting_position, self.processor.tip_pose[0:3, 3], threshold = 0.5
                            )
                            
                            if not moved_ap:
                                print( "deadzone, GOING OUT...." )
                                while not moved_ap:
                                    self.communicator.send_steps_to_arduino_dead_zone(delta_steps=ap)
                        
                        
                                    moved_ap = self.container.detect_dead_zone(
                                        scaled_actuations, starting_position, self.processor.tip_pose[0:3, 3]
                                    )
                                #if scaled_actuations[2] == -self.max_ap_per_step:
                                    
                                   #self.workspace_mapper.actual_AP_actuation = self.workspace_mapper.start_P_dead - self.max_ap_per_step    
                                #elif scaled_actuations[2] == -self.max_ap_per_step:
                                    #self.workspace_mapper.actual_AP_actuation = self.workspace_mapper.start_A_dead + self.max_ap_per_step  
                                     
                            else:
                                print("false positive  !!!!!")        
                                scaled_actuations[2] = 2 * scaled_actuations[2] 
                                print("new scaled actuations for AP:", scaled_actuations[2])
                        actual_pos =  self.processor.tip_pose[0:3, 3]
                        
                        self.communicator.send_steps_to_arduino_dead_zone(delta_steps = rest)
                        
                        
                        moved = self.container.detect_dead_zone(
                                        scaled_actuations, actual_pos, self.processor.tip_pose[0:3, 3]
                                    )
                        
                        if moved:
                            self.container.update_observations(
                                scaled_actuations, starting_position, self.processor.tip_pose[0:3, 3], 0
                            )
                        
                        
                        
                        
                        
                        self.workspace_mapper.get_catheter_state()
                        self.workspace_mapper.add_insertion(scaled_actuations[0])
                        self.workspace_mapper.add_step_AP(scaled_actuations[2])
                        self.workspace_mapper.add_step_ML(scaled_actuations[1])
                    else:
                        self.communicator.send_steps_to_arduino_dead_zone(delta_steps=scaled_actuations)
                        self.workspace_mapper.add_insertion(scaled_actuations[0])
                        self.workspace_mapper.add_step_AP(scaled_actuations[2])
                        self.workspace_mapper.add_step_ML(scaled_actuations[1])
                        
                        self.workspace_mapper.get_catheter_state()
                        moved = self.container.detect_dead_zone(
                            scaled_actuations, starting_position, self.processor.tip_pose[0:3, 3]
                        )

                        if moved:
                            self.container.update_observations(
                                scaled_actuations, starting_position, self.processor.tip_pose[0:3, 3], 0
                            )


                if np.allclose(remaining, 0):
                    break  # stop if fully done

            # --- After actuation, log results ---
            data_row.update({
                "reached_tip_pos_x": self.processor.tip_pose[0, 3],
                "reached_tip_pos_y": self.processor.tip_pose[1, 3],
                "reached_tip_pos_z": self.processor.tip_pose[2, 3],
                "position_error": np.linalg.norm(self.processor.tip_pose[:3, 3] - target),
                "target_error": self.processor.target_error,
                "normalized_position_error": np.linalg.norm(
                    (self.processor.tip_pose[:3, 3] - target) / norm_of_the_displacement
                ),
                "min_target_error": self.processor.min_target_error
            })

            with open(self.processor.run_paths["results_csv"], "a", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=self.processor.run_paths["fieldnames_dict"])
                writer.writerow(data_row)

            if self.estimate_jacobian:
                if not (self.container.is_filling and self.container.current_index == 0):
                    self.jacobian_estimator.estimate_jacobian_lyapunof()




    def go_out_of_deadzone_P_to_A(self, resolution: int = 50):
        
        # when we are in posterior workspace and we want to actuate anteriorly we first need to go out of deadzone
        steps = math.ceil(abs(self.workspace_mapper.start_A_dead - self.workspace_mapper.start_P_dead) / resolution)
        for i in range(steps):
            actuation = np.array((0, 0, resolution))
            self.communicator.send_steps_to_arduino_dead_zone(delta_steps=actuation)
            
        # now we search for valid movements
        ap_valid_count = 0
            
            
        while ap_valid_count < 2:   
            print("GETTING OUT OF AP")   
            starting_position = self.processor.tip_pose[0:3, 3].copy() 
            self.communicator.send_steps_to_arduino_dead_zone(delta_steps=np.array(([0, 0, 15]))) 
            moved = self.container.detect_dead_zone(
                    np.array(([0, 0, 15])), starting_position, self.processor.tip_pose[0:3, 3]
                )
            if moved:
                ap_valid_count += 1
            else:
                ap_valid_count = 0
                
        self.workspace_mapper.actual_AP_actuation = self.workspace_mapper.start_A_dead
        self.workspace_mapper.ap_state = -1
          
    def go_out_of_deadzone_A_to_P(self, resolution: int = 50):
        
        # when we are in anterior workspace and we want to actuate posteriorly we first need to go out of deadzone
        steps = math.ceil(abs(self.workspace_mapper.start_A_dead - self.workspace_mapper.start_P_dead) / resolution)
        for i in range(steps):
            actuation = np.array((0, 0, -resolution))
            self.communicator.send_steps_to_arduino_dead_zone(delta_steps=actuation)
            
        # now we search for valid movements
        ap_valid_count = 0
            
            
        while ap_valid_count < 2:   
            print("GETTING OUT OF AP")   
            starting_position = self.processor.tip_pose[0:3, 3].copy() 
            self.communicator.send_steps_to_arduino_dead_zone(delta_steps=np.array(([0, 0, -15]))) 
            moved = self.container.detect_dead_zone(
                    np.array(([0, 0, -15])), starting_position, self.processor.tip_pose[0:3, 3]
                )
            if moved:
                ap_valid_count += 1
            else:
                ap_valid_count = 0      
        self.workspace_mapper.actual_AP_actuation = self.workspace_mapper.start_P_dead
        self.workspace_mapper.ap_state = 1
        
        
        
        



    def go_out_of_deadzone(self):
        ml_valid_count = 0
        ap_valid_count = 0   
        starting_position = self.processor.tip_pose[0:3, 3].copy() 
        '''
        for i in range(10):
            rp.sleep(0.1)
            
            self.container.detect_dead_zone(
                    np.array(([0, 0, -15])), starting_position, self.processor.tip_pose[0:3, 3]
                )
            starting_position = self.processor.tip_pose[0:3, 3]
            
        '''    
        while ap_valid_count < 2:   
            print("GETTING OUT OF AP")   
            starting_position = self.processor.tip_pose[0:3, 3].copy() 
            self.communicator.send_steps_to_arduino_dead_zone(delta_steps=np.array(([0, 0, -15]))) 
            moved = self.container.detect_dead_zone(
                    np.array(([0, 0, -15])), starting_position, self.processor.tip_pose[0:3, 3]
                )
            if moved:
                ap_valid_count += 1
            else:
                ap_valid_count = 0
                
        '''
        #### WAIT FOR ACTUATIONS TO AVOID CROSS TALK and buffer it ###
        
        for i in range(10):
            rp.sleep(0.1)
            
            self.container.detect_dead_zone(
                    np.array(([0, -20, 0])), starting_position, self.processor.tip_pose[0:3, 3]
                )
            starting_position = self.processor.tip_pose[0:3, 3]
        '''    
        while ml_valid_count < 2:   
            print("GETTING OUT OF ML")   

            starting_position = self.processor.tip_pose[0:3, 3].copy()    
            self.communicator.send_steps_to_arduino_dead_zone(delta_steps=np.array(([0, -20, 0]))) 
            moved = self.container.detect_dead_zone(
                    np.array(([0, -20, 0])), starting_position, self.processor.tip_pose[0:3, 3]
                )
            
            
            if moved:
                ml_valid_count += 1
            else:
                ml_valid_count = 0    
        

     

    def loop_control_to_waypoint(self, N):
        try:
            for i in range(N):
                
                if getattr(self, "plotter", None) and self.plotter.stop_requested:
                    rp.loginfo("Stop requested; breaking control loop.")
                    break
                
                if np.linalg.norm(self.processor.tip_pose[:3, 3] - self.waypoint) >= 5.0:
                    print("I-th iteration", i)
                    # stop if requested (from 't')
                    if getattr(self, "plotter", None) and self.plotter.stop_requested:
                        rp.loginfo("Stop requested; breaking control loop.")
                        break
                    
                    

                    # -------- body --------
                    predictions_list = []
                    time = rp.get_time() - self.processor.start_time
                    for _ in range(N - 3 + 1):
                        pred = self.waypoint
                        predictions_list.append(pred)
                        time += self.delta_t_btw_actuations
                        
                    predictions_array = np.asarray(predictions_list)

                    actual_position = self.processor.tip_pose.copy()
                    #print("Passing the value to the MPC function: ", actual_position)

                    if self.dynamical_weight_update:
                        self.trajectory_estimator.build_problem_dynamical_weight(
                            prediction_horizon=N - 3, 
                            actual_position=actual_position,
                            target_position=predictions_array,
                            len_inserted = self.workspace_mapper.actual_insertion
                        )
                        print("Updating weights dynamically!!!!\r\n")
                    else:
                        self.trajectory_estimator.build_problem(
                            prediction_horizon=N - 3,
                            actual_position=actual_position,
                            target_position=predictions_array,
                        )
                        
                    self.trajectory_estimator.solve()
                    self.publish_trajectory_points(self.trajectory_estimator.X_sol)

                    tip = self.processor.tip_pose[:3, 3].copy()

                    norm = np.linalg.norm(tip - self.waypoint)
                    print("Position error: ", norm)

                    delta = self.trajectory_estimator.X_sol[1] - self.processor.tip_pose[:3, 3].copy()
                    print("SOLUTION ",self.trajectory_estimator.X_sol)
                    if self.estimate_jacobian:
                        J = self.jacobian_estimator.actual_jac
                        dl = J @ delta
                        dl = np.round(dl).astype(int)
                        
                    else:
                        J = self.jacobian_estimator.compute_analytical_jacobian(tip[0], tip[1], tip[2])
                        self.jacobian_estimator.container.previous_jacobians.append(J)
                        u = self.pid_controller.compute_control_action(self.trajectory_estimator.X_sol[1], tip)
                        dl = J @ u
                        dl = np.round(dl * np.array(self.dl_to_steps)).astype(int)
                        print("Next actuation step for PID:", dl)

                    self.check_for_actuation_bounds(dl)
                    #self.actuate_in_smaller_steps(dl, self.trajectory_estimator.X_sol[1], self.waypoint)                  
                            
                        

        except Exception as e:
            # any unexpected error -> finalize if possible
            raise_with_line_info(e)
            if getattr(self, "plotter", None):
                self.plotter.stop_requested = True
                try:
                    self.plotter.finalize_one_shot(self.trajectory_estimator.workspace_mapper)
                finally:
                    return
            rp.signal_shutdown("Unhandled exception (no plotter)")
        finally:
            # If we exited because 't' was pressed, run finalize.
            if getattr(self, "plotter", None) and self.plotter.stop_requested:
                self.plotter.finalize_one_shot(self.trajectory_estimator.workspace_mapper)
    
    
    def loop_control_to_target(self):
        try:
            while not rp.is_shutdown():
                # stop if requested (from 't')
                if getattr(self, "plotter", None) and self.plotter.stop_requested:
                    rp.loginfo("Stop requested; breaking control loop.")
                    break
                 
                # original termination condition
                if np.linalg.norm(self.processor.tip_pose[:3, 3] - self.processor.valve_pose[:3, 3]) <= 0.5:
                    rp.loginfo("Target reached; finalizing control loop.")
                    if getattr(self, "plotter", None):
                        # mark as stopped (so any outer logic knows), then finalize visuals/rewind/etc.
                        self.plotter.stop_requested = True
                        try:
                            self.plotter.finalize_one_shot(self.trajectory_estimator.workspace_mapper)
                        finally:
                            return  # leave Compute_MPC_Control cleanly
                    else:
                        rp.signal_shutdown("Target reached")
                        return

                # -------- body --------
                predictions_list = []
                valve = self.processor.valve_pose[:3, 3].copy()
                time = rp.get_time() - self.processor.start_time 
                for _ in range(self.Number_of_iteration + 1):
                    if self.processor.model_identified:
                        pred =  self.valve_base + self.kalman_predictor.predict_no_update(time) * self.valve_motion_direction
                    else:
                        pred = self.processor.move_valve(self.processor.valve_amplitude, self.processor.valve_frequency, valve, time, self.processor.valve_dir)
                    pred = self.valve_base
                    predictions_list.append(pred)
                    time += self.delta_t_btw_actuations
                    #print(" TIME FOUND", time)
                predictions_array = np.asarray(predictions_list)

                actual_position = self.processor.tip_pose.copy()
                print(" Passing the value to the MPC function: ", predictions_array)
                starting_computation_time = rp.get_time()
                if self.dynamical_weight_update:
                    self.trajectory_estimator.build_problem_dynamical_weight(
                        prediction_horizon=self.Number_of_iteration,
                        actual_position=actual_position,
                        target_position=predictions_array,
                        len_inserted = self.workspace_mapper.actual_insertion
                    )
                    print("Updating weights dynamically!!!!\r\n")
                else:
                    self.trajectory_estimator.build_problem_classic_MPC(
                        prediction_horizon=self.Number_of_iteration,
                        actual_position=actual_position,
                        target_position=predictions_array,
                    )
                   
                self.trajectory_estimator.solve()
                self.publish_trajectory_points(self.trajectory_estimator.X_sol)    
                #print(self.trajectory_estimator.X_sol)  
                tip = self.processor.tip_pose[:3, 3].copy()
                #print("tip position from aurora during mpc: \r\n", tip)


                #print("posizione del target: \r\n", valve)
                norm = np.linalg.norm(tip - self.processor.valve_pose[:3, 3].copy())
                print("Position error: \r\n", norm)
                print("time elapsed:",rp.get_time() - self.processor.start_time)
                delta = self.trajectory_estimator.X_sol[1] - self.processor.tip_pose[:3, 3].copy()
                if self.estimate_jacobian:
                    J = self.jacobian_estimator.actual_jac

                    dq_vec = np.round(J @ delta).astype(int)
                    #if not self.dynamical_weight_update:
                        #dq_vec = np.round(self.trajectory_estimator.X_sol[0]).astype(int)
                    print("Next actuation step:", dq_vec)

                    dz_flag = self.workspace_mapper.prune_actuations_no_dead(dq_vec)
                    print("deadzone flag:", dz_flag)

                    if dz_flag == 1:
                        print("GETTING OUT OF P TO A DZ")
                        #self.go_out_of_deadzone_P_to_A()

                        dx = self.trajectory_estimator.X_sol[1] - self.processor.tip_pose[:3, 3].copy()
                        dq_vec = np.round(J @ dx).astype(int)

                    elif dz_flag == -1:
                        print("GETTING OUT OF A TO P DZ")
                        #self.go_out_of_deadzone_A_to_P()

                        dx = self.trajectory_estimator.X_sol[1] - self.processor.tip_pose[:3, 3].copy()
                        dq_vec = np.round(J @ dx).astype(int)

                    # 🔒 GARANZIA: dq_vec è sempre un vettore
                    dq = dq_vec

                    
                    
                else:
                    print(self.pid_controller.use_pid)
                    if self.pid_controller.use_pid == True:
                        J = self.jacobian_estimator.compute_analytical_jacobian(tip[0], tip[1], tip[2])
                        self.jacobian_estimator.container.previous_jacobians.append(J)
                        u = self.pid_controller.compute_control_action(self.trajectory_estimator.X_sol[1], tip)
                        dl = J @ u
                        dq = np.round(dl * np.array(self.dl_to_steps)).astype(int)                        
                        print("Next actuation step for PID:", dq)
                        
                    else:
                        dl = self.trajectory_estimator.X_sol[0]
                        dq = np.round(dl * np.array(self.dl_to_steps)).astype(int)
                        print("Next actuation step for classic MPC:", dq)
                    
                
                
                


                
                
                
                         
                self.check_for_actuation_bounds(dq)
                self.actuate_in_smaller_steps_ins_first_hysteresis(dq, self.trajectory_estimator.X_sol[1], valve)
                #self.communicator.send_steps_to_arduino_dead_zone(delta_steps=dl)
                self.delta_t_btw_actuations = rp.get_time() - starting_computation_time
                    

                    

        except Exception as e:
            # any unexpected error -> finalize if possible
            raise_with_line_info(e)
            if getattr(self, "plotter", None):
                self.plotter.stop_requested = True
                try:
                    self.plotter.finalize_one_shot(self.trajectory_estimator.workspace_mapper)
                finally:
                    return
            rp.signal_shutdown("Unhandled exception (no plotter)")
        finally:
            # If we exited because 't' was pressed, run finalize.
            if getattr(self, "plotter", None) and self.plotter.stop_requested:
                self.plotter.finalize_one_shot(self.trajectory_estimator.workspace_mapper)
        
        
                                 
    def Compute_MPC_Control(self):
        
        self.estimate_jacobian_if_necessary()
        
        self.calculate_tf_for_valve()
        print("SLEEPING 5 SECONDS, START THE MOTOR")
        rp.sleep(5)
        print("SAMPLING...")
        self.processor.wait_for_sync = self.identify_model()        
        #self.generate_waypoint()
        self.go_out_of_deadzone()
        print("actual workspace state: ")
        self.workspace_mapper.get_catheter_state()
        #rp.sleep(5)
        self.processor.start_time = rp.get_time()
        #print( " STARTING THE CONTROL TO WAYPOINT ...")
        #self.loop_control_to_waypoint(N = 8)

        print ( " STARTING THE CONTROL TO TARGET ...")
        self.processor.control_loop_started = True
        self.loop_control_to_target()

        

    def publish_waypoint(self, waypoint, frame_id):
        if not hasattr(self, 'waypoint_pub'):
            self.waypoint_pub = rp.Publisher('/waypoint', PointStamped, queue_size=1)
        point_msg = PointStamped()
        point_msg.header = Header()
        point_msg.header.frame_id = "world"
        point_msg.header.stamp = rp.Time.now()
        
        point_msg.point.x = waypoint[0] / 10
        point_msg.point.y = waypoint[1] / 10
        point_msg.point.z = waypoint[2] / 10
        print("publishing point...")
        self.waypoint_pub.publish(point_msg)
        print(f"Waypoint being published: x={point_msg.point.x}, y={point_msg.point.y}, z={point_msg.point.z}")

            
    def publish_trajectory(self, trajectory):
        if not hasattr(self, 'path_pub'):
            self.path_pub = rp.Publisher('/trajectory_path', Path, queue_size=10)

        if trajectory is None or len(trajectory) == 0:
            rp.logwarn("No trajectory to publish")
            return

        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = rp.Time.now()
        path_msg.header.frame_id = "world"  

        for point in trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0] / 10
            pose.pose.position.y = point[1] / 10
            pose.pose.position.z = point[2] / 10

            # No orientation
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        
        
    def publish_trajectory_points(self, trajectory):
        if not hasattr(self, 'points_pub'):
            self.points_pub = rp.Publisher('/trajectory_points', Marker, queue_size=10)

        if trajectory is None or len(trajectory) == 0:
            rp.logwarn("No trajectory points to publish")
            return

        # Creating the marker message
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = rp.Time.now()
        marker.header.frame_id = "world"  
        marker.ns = "trajectory_points"
        marker.id = 0
        marker.type = Marker.POINTS   # <–– shows points, not lines
        marker.action = Marker.ADD

        # Point appearance
        marker.scale.x = 0.05   # point width (in meters)
        marker.scale.y = 0.05   # point height
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # RGBA (red, opaque)

        # Adding points
        SCALE = 0.1  # scales by 1/10 for Rviz visibility
        for p in trajectory:
            pt = Point()
            pt.x = p[0] * SCALE
            pt.y = p[1] * SCALE
            pt.z = p[2] * SCALE
            marker.points.append(pt)

        self.points_pub.publish(marker)
        
        
    def set_controller_use_previously_estimated_jacobian(self, value: bool):
        # Path to config.yaml
        config_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'config')
        yaml_path = os.path.join(config_dir, 'config.yaml')

        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f'{yaml_path} does not exist')

        # Load existing YAML
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        # Defensive checks
        if 'controller' not in config:
            raise KeyError('controller section missing in config.yaml')

        config['controller'].setdefault('attributes', {})

        # Update only the calibrated flag
        config['controller']['attributes']['use_previously_estimated_jacobian'] = value

        # Save back
        with open(yaml_path, 'w') as f:
            yaml.safe_dump(config, f, sort_keys=False)

        print(f"'controller.attribtes.use_previously_estimated_jacobian' set to {value} in config.yaml")