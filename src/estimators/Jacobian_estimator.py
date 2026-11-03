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
import json
import os


class JacobianEstimator:
    def __init__(self, jacobian_shape, container, store_previous = False):
        """Initialize the Jacobian estimator with default values."""
        self.previous_jac = np.zeros((jacobian_shape,jacobian_shape))  # Stores the previous Jacobian estimate (matrix)
        self.actual_jac = np.eye(jacobian_shape)    # Current estimated Jacobian
        self.container = container
        self.store_previous = store_previous
        self.catheter_diameter = 3
        self.tolerance = 1e-5
        self.max_iter = 100
        self.i = 0
    

    def validate_and_finalize(self):
        self.tolerance = float(self.tolerance)
    
    def update_process_disturbance(self):
        """Updates the process disturbance in a circular way."""
        for i in range(self.container.horizon_size):
            index = (self.container.current_index + i) % self.container.horizon_size  # Circular indexing
            self.container.disturbance[index] = (
                self.container.beta *
                (self.container.p_target[index] - self.container.p_meas[index]) 
            )

    def generate_CC_points(self, tip_pose, k, theta, phi, l, N):
        """
        Generates points along a constant curvature arc, expressed in the base frame.
        
        Parameters:
        - tip_pose: 4x4 homogeneous matrix of the tip in the base frame
        - k, theta, phi, l: curvature parameters (expressed in the tip frame)
        - N: number of segments

        Returns:
        - np.array of shape (N+1, 3): points in the base frame
        """
        points = []
        
        # Extract rotation and translation from tip_pose
        R_tip = tip_pose[:3, :3]
        t_tip = tip_pose[:3, 3]
        '''
        print("Tip position (homogeneous matrix):\n", tip_pose)
        print(f"Curvature parameters:")
        print(f"  k (curvature)   = {k:.6f}")
        print(f"  theta (angle)   = {theta:.6f} rad ({np.degrees(theta):.2f} deg)")
        print(f"  phi (orientation) = {phi:.6f} rad ({np.degrees(phi):.2f} deg)")
        print(f"  l (arc length)  = {l:.6f}")
        print(f"Number of segments: {N}")
        '''
        for s in np.linspace(0, l, N + 1):
            #print(f"\ns = {s:.6f}")
            ratio = s / l if l != 0 else 0.0
            theta_s = theta * ratio

            # Point in the tip's local frame (using CC model)
            x = (1 / k) * np.cos(phi) * (1 - np.cos(theta_s))
            y = (1 / k) * np.sin(phi) * (1 - np.cos(theta_s))
            z = (1 / k) * np.sin(theta_s)
            local_point = np.array([x, y, z])

            #print(f"Local point (tip frame): [{x:.6f}, {y:.6f}, {z:.6f}]")

            # Transform to base frame
            global_point = R_tip @ local_point + t_tip
            #print(f"Global point (base frame): [{global_point[0]:.6f}, {global_point[1]:.6f}, {global_point[2]:.6f}]")
            
            points.append(global_point)
            
        #print("\nAll points on the curvature (base frame):")
        #print(np.array(points))
        return np.array(points)

    def compute_CC_model(self, tip):
        d = 3  # diametro del catetere (mm)

        point = np.array(tip) 
        x, y, z = point

        # Calcolo dei parametri dell'arco
        phi = math.atan2(y, x)
        k = 2 * math.sqrt(x**2 + y**2) / (x**2 + y**2 + z**2)
        theta = math.acos(1 - k * math.sqrt(x**2 + y**2))
        l = theta / k if k != 0 else 0

        # Spostamenti dei tendini
        q_inv = np.zeros(3)
        q_inv[0] = l 
        q_inv[1] = - d / 2 * theta * math.cos(math.pi / 2 - phi)
        q_inv[2] = - d / 2 * theta * math.cos(-phi)

        return l, phi, k, theta, q_inv

    
    '''

    COMPUTE JACOBIAN FUNCTION FOR SIMPLE CLOSED LOOP CONROLLER

    '''


    def compute_analytical_jacobian(self,x_meas,y_meas,z_meas):
        # Define your symbolic variables
        x, y, z = sp.symbols('x y z')
        d = self.catheter_diameter  # diameter of the catehetr (mm)
        # Computing ARC parameters

        phi = sp.atan2(y, x)
        k = 2 * sp.sqrt(x ** 2 + y ** 2) / (x ** 2 + y ** 2 + z ** 2)
        theta = sp.acos(1 - k * sp.sqrt(x ** 2 + y ** 2))
        l = theta / k  # (mm)

        # computing Tendons displacement

        q_inv = sp.zeros(3, 1)
        q_inv[0] = l # inserzione
        q_inv[1] = - d / 2 * theta * sp.cos(math.pi/ 2 - phi) # ML ## potrebbe dover essere da cambiare il segno!!
        q_inv[2] = - d / 2 * theta * sp.cos(- phi) # AP  ## potrebbe dover essere da cambiare il segno!!

        #q_inv[3] = -d / 2 * theta * sp.cos(3 * sp.pi / 2 - phi)

        # Compute the Jacobian matrix
        J = sp.Matrix(q_inv).jacobian([x, y, z])
        # Substitute the current position into the Jacobian
        J_numeric = J.subs({x: x_meas, y: y_meas, z: z_meas})

        #self.previous_jac = self.actual_jac  # Store previous value
        #self.actual_jac = np.array(J_numeric).astype(np.float64)
        return np.array(J_numeric).astype(np.float64)

    '''

        BUILD JACOBIAN FUNCTIONS FOR casadi


    '''



    def build_jacobian_fun(self):
        x = ca.MX.sym("x")
        y = ca.MX.sym("y")
        z = ca.MX.sym("z")
        vars_vec = ca.vertcat(x, y, z)

        d = self.catheter_diameter 
        phi = ca.atan2(y, x)
        k = 2 * ca.sqrt(x**2 + y**2) / (x**2 + y**2 + z**2)
        theta = ca.acos(1 - k * ca.sqrt(x**2 + y**2))
        l = theta / k

        q_inv = ca.vertcat(
            l, # insertion
            - d / 2 * theta * ca.cos(ca.pi / 2 - phi), # ML  ## potrebbe dover essere da cambiare il segno!!
            - d / 2 * theta * ca.cos(-phi) # AP   ## potrebbe dover essere da cambiare il segno!!
            
        )

        J = ca.jacobian(q_inv, vars_vec)

        return ca.Function("jacobian_fun", [vars_vec], [J])
    
    
    
    
    def build_Inverse_jacobian_fun(self):
        x = ca.MX.sym("x")
        y = ca.MX.sym("y")
        z = ca.MX.sym("z")
        vars_vec = ca.vertcat(x, y, z)

        d = self.catheter_diameter 
        phi = ca.atan2(y, x)
        k = 2 * ca.sqrt(x**2 + y**2) / (x**2 + y**2 + z**2)
        theta = ca.acos(1 - k * ca.sqrt(x**2 + y**2))
        l = theta / k

        q_inv = ca.vertcat(
            l, # insertion
            - d / 2 * theta * ca.cos(ca.pi / 2 - phi), # ML  ## potrebbe dover essere da cambiare il segno!!
            - d / 2 * theta * ca.cos(-phi) # AP   ## potrebbe dover essere da cambiare il segno!!
            
        )

        J = ca.inv(ca.jacobian(q_inv, vars_vec))

        return ca.Function("jacobian_fun", [vars_vec], [J])

    
    '''

    FUNCTION TO ESTIMATE THE JACOBIAN BY OPTIMIZATION ON PREVIOUS DATA 
 
    '''   
    
    def initialize_jacobian(self):
        iteration = 0
        
        while (scipy.linalg.norm(self.actual_jac - self.previous_jac, 'fro') > 1e-2 and iteration < 1000) or iteration < 1:
             
            # Definisci variabili per l'ottimizzazione
            
            delta_j = cp.Variable(self.actual_jac.shape)
            j_k1 = self.actual_jac + delta_j
            error_matrix = self.container.actuations - (j_k1 @ self.container.dp.T).T  # (N, 3)
            # Calcolo della funzione obiettivo
            raw_norms_squared = cp.sum(cp.square(error_matrix), axis=1)  # (N,)


            weighted_sum = cp.sum(raw_norms_squared)

            objective = cp.Minimize(weighted_sum + cp.norm(delta_j, 2))
            problem = cp.Problem(objective, [])

            problem.solve()

            iteration += 1

            if j_k1.value is not None:
                self.previous_jac = self.actual_jac
                self.actual_jac = j_k1.value
            else:
                print(f"Optimization failed at iteration {iteration}, status: {problem.status}")
                return 0
        print(f"Optimization successful at iteration: {iteration}")
        print(f"Computed Jacobian: {self.actual_jac}")
        if self.store_previous:
          self.container.previous_jacobians.append(self.actual_jac)

        return 1  # Successo
    


    def estimate_jacobian_dead_zone(self):

        iteration = 0
        path_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "experiments", "to_evaluate")
        )
        latest_file = os.path.join(path_dir, "LATEST")

        # Read the content (the experiment path)
        with open(latest_file, "r") as f:
            latest_experiment_path = f.read().strip()

        #print(f"📁 Latest experiment path: {latest_experiment_path}")
        
        latest_experiment_path = os.path.join(latest_experiment_path, "logs")
        
        
        while (scipy.linalg.norm(self.actual_jac - self.previous_jac, 'fro') > self.tolerance and iteration < self.max_iter) or iteration < 1:
            
            
            
            if scipy.linalg.norm(self.actual_jac - self.previous_jac, 'fro') > 20:
                self.container.save_state(directory=latest_experiment_path, filename=f"state_{self.i}.json")
                self.i += 1
            
            
            # Estrai dati secondo il tipo di riempimento
            if self.container.is_filling:
                indices = range(self.container.current_index)
                dp = self.container.dp[:self.container.current_index]
                actuations = self.container.actuations[:self.container.current_index]
                rho = np.flip(self.container.rho[:self.container.current_index,0])
            else:
                horizon = self.container.horizon_size
                indices = [(self.container.current_index + i) % horizon for i in range(horizon)]
                dp = self.container.dp[indices]
                actuations = self.container.actuations[indices]
                rho = np.roll(self.container.rho, -self.container.current_index)
                rho = np.mean(rho,axis = 1)
            # Aggiorna errori e pesi rho
            
            for idx, i in enumerate(indices):
                error_vec = (
                     self.container.actuations[i]
                    - self.actual_jac @ self.container.dp[i]
                    - self.container.disturbance[i]
                )
                self.container.errors[i] = np.linalg.norm(error_vec, 2)
                self.container.rho[i] = (
                    (self.container.lambda_forget ** idx)
                    * ( self.container.sigma_forget ** 2)
                    / (self.container.errors[i] ** 2 + self.container.sigma_forget ** 2)
                )
                
            
            # Definisci variabili per l'ottimizzazione
            delta_j = cp.Variable(self.actual_jac.shape)
            j_k1 = self.actual_jac + delta_j
            error_matrix = actuations - (j_k1 @ dp.T).T  # (N, 3)
            
            
            # Calcolo della funzione obiettivo
            row_norms_squared = cp.sum(cp.square(error_matrix), axis=1)  # (N,)


            weighted_errors = cp.multiply(rho, row_norms_squared)
            weighted_sum = cp.sum(weighted_errors)

            objective = cp.Minimize(weighted_sum + cp.norm(delta_j, 'fro'))
            problem = cp.Problem(objective, [])
            
            problem.solve()

            iteration += 1

            if j_k1.value is not None:
                self.previous_jac = self.actual_jac
                self.actual_jac = j_k1.value
            else:
                print(f"Optimization failed at iteration {iteration}, status: {problem.status}")
                return 0
        
        #print(f"Optimization successful at iteration: {iteration}")
        #print(f"Computed Jacobian: {self.actual_jac}")
        if self.store_previous:
            self.container.previous_jacobians.append(self.actual_jac)

        return 1  # Successo
    
    
    
    
    def estimate_jacobian_lyapunof(self):

        iteration = 0
        path_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", "experiments", "to_evaluate")
        )
        latest_file = os.path.join(path_dir, "LATEST")

        # Read the content (the experiment path)
        with open(latest_file, "r") as f:
            latest_experiment_path = f.read().strip()

        #print(f"📁 Latest experiment path: {latest_experiment_path}")
        
        latest_experiment_path = os.path.join(latest_experiment_path, "logs")
        
        
        while (scipy.linalg.norm(self.actual_jac - self.previous_jac, 'fro') > self.tolerance and iteration < self.max_iter) or iteration < 1:
            
            
            
            if scipy.linalg.norm(self.actual_jac - self.previous_jac, 'fro') > 20:
                self.container.save_state(directory=latest_experiment_path, filename=f"state_{self.i}.json")
                self.i += 1
            
            
            # Estrai dati secondo il tipo di riempimento
            if self.container.is_filling:
                indices = range(self.container.current_index)
                dp = self.container.dp[:self.container.current_index]
                actuations = self.container.actuations[:self.container.current_index]
                rho = np.flip(self.container.rho[:self.container.current_index,0])
            else:
                horizon = self.container.horizon_size
                indices = [(self.container.current_index + i) % horizon for i in range(horizon)]
                dp = self.container.dp[indices]
                actuations = self.container.actuations[indices]
                rho = np.roll(self.container.rho, -self.container.current_index)
                rho = np.mean(rho,axis = 1)
            # Aggiorna errori e pesi rho
            
            for idx, i in enumerate(indices):
                error_vec = (
                     self.container.actuations[i]
                    - self.actual_jac @ self.container.dp[i]
                    - self.container.disturbance[i]
                )
                self.container.errors[i] = np.linalg.norm(error_vec, 2)
                self.container.rho[i] = (
                    (self.container.lambda_forget ** idx)
                    * ( self.container.sigma_forget ** 2)
                    / (self.container.errors[i] ** 2 + self.container.sigma_forget ** 2)
                )
                
            
            # Definisci variabili per l'ottimizzazione
            delta_j = cp.Variable(self.actual_jac.shape)
            j_k1 = self.actual_jac + delta_j
            error_matrix = actuations - (j_k1 @ dp.T).T  # (N, 3)
            
            
            # Calcolo della funzione obiettivo
            row_norms_squared = cp.sum(cp.square(error_matrix), axis=1)  # (N,)


            weighted_errors = cp.multiply(rho, row_norms_squared)
            weighted_sum = cp.sum(weighted_errors)

            objective = cp.Minimize(weighted_sum + cp.norm(delta_j, 'fro'))
            problem = cp.Problem(objective, [])
            
            problem.solve()

            iteration += 1

            if j_k1.value is not None:
                self.previous_jac = self.actual_jac
                self.actual_jac = j_k1.value
            else:
                print(f"Optimization failed at iteration {iteration}, status: {problem.status}")
                return 0
        
        #print(f"Optimization successful at iteration: {iteration}")
        #print(f"Computed Jacobian: {self.actual_jac}")
        if self.store_previous:
            self.container.previous_jacobians.append(self.actual_jac)

        return 1  # Successo
    
    
    def estimate_jacobian_pseudoinverse(self):
        """
        Stima la Jacobiana J come dq/dx usando la pseudoinversa:
        J = actuations.T @ pinv(dp.T)
        """
        # -------------------------------------------------------
        # 1. Estrai dati dal buffer
        # -------------------------------------------------------
        if self.container.is_filling:
            # Buffer non pieno
            indices = range(self.container.current_index)
            dp = self.container.dp[:self.container.current_index]
            actuations = self.container.actuations[:self.container.current_index]
        else:
            # Buffer pieno, modalità circolare
            horizon = self.container.horizon_size
            indices = [(self.container.current_index + i) % horizon for i in range(horizon)]
            dp = self.container.dp[indices]
            actuations = self.container.actuations[indices]

            # Controllo dimensioni
            if dp.shape[0] == 0:
                print("Buffer vuoto: impossibile stimare la Jacobiana")
                return None

            # -------------------------------------------------------
            # 2. Calcola la pseudoinversa
            # -------------------------------------------------------
            # J = actuations.T @ pinv(dp.T)
            J_est = actuations.T @ np.linalg.pinv(dp.T)

            # Aggiorna lo stato
            self.previous_jac = self.actual_jac
            self.actual_jac = J_est

            if getattr(self, "store_previous", False):
                self.container.previous_jacobians.append(J_est)

            print("✅ Jacobiana stimata con pseudoinversa")
            print(J_est)
            return 1

    def estimate_jacobian_decouple(self):

        iteration = 0

        # -------------------------------------------------------
        # 1. Load latest experiment path
        # -------------------------------------------------------
        path_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", "experiments", "to_evaluate")
        )
        latest_file = os.path.join(path_dir, "LATEST")

        with open(latest_file, "r") as f:
            latest_experiment_path = f.read().strip()

        print(f"📁 Latest experiment path: {latest_experiment_path}")

        latest_experiment_path = os.path.join(latest_experiment_path, "logs")

        # -------------------------------------------------------
        # Loop until convergence of J
        # -------------------------------------------------------
        while (scipy.linalg.norm(self.actual_jac - self.previous_jac, 'fro') > self.tolerance 
            and iteration < self.max_iter) or iteration < 1:

            # Save state if jump in Jacobian is excessive
            if scipy.linalg.norm(self.actual_jac - self.previous_jac, 'fro') > 20:
                self.container.save_state(directory=latest_experiment_path, filename=f"state_{self.i}.json")
                self.i += 1

            # -------------------------------------------------------
            # 2. Extract buffer contents
            # -------------------------------------------------------
            if self.container.is_filling:
                indices = range(self.container.current_index)
                dp = self.container.dp[:self.container.current_index]
                actuations = self.container.actuations[:self.container.current_index]
                rho = np.flip(self.container.rho[:self.container.current_index, 0])

            else:
                # Circular buffer mode
                horizon = self.container.horizon_size
                indices = [(self.container.current_index + i) % horizon for i in range(horizon)]
                
                dp = self.container.dp[indices]
                actuations = self.container.actuations[indices]

                rho = np.roll(self.container.rho, -self.container.current_index)
                rho = np.mean(rho, axis=1)   # flatten to vector

            # -------------------------------------------------------
            # 3. Update rho weighting with forgetting + dead zone error
            # -------------------------------------------------------
            for idx, i in enumerate(indices):
                error_vec = (
                    self.container.actuations[i]
                    - self.actual_jac @ self.container.dp[i]
                    - self.container.disturbance[i]
                )
                self.container.errors[i] = np.linalg.norm(error_vec, 2)

                self.container.rho[i] = (
                    (self.container.lambda_forget ** idx)
                    * (self.container.sigma_forget ** 2)
                    / (self.container.errors[i] ** 2 + self.container.sigma_forget ** 2)
                )
                self.container.rho[i] = 1

            # -------------------------------------------------------
            # 4. CVXPY optimization variables
            # -------------------------------------------------------
            delta_j = cp.Variable(self.actual_jac.shape)
            j_k1 = self.actual_jac + delta_j

            # prediction: (N, 3)
            error_matrix = actuations - (j_k1 @ dp.T).T

            # -------------------------------------------------------
            # 5. Decouple the three directional cost components
            # -------------------------------------------------------
            e_ins = error_matrix[:, 0]   # INS
            e_ml  = error_matrix[:, 1]   # ML
            e_ap  = error_matrix[:, 2]   # AP

            # Directional weights (user-defined or default = 1.0)
            w_ins = getattr(self, "w_ins", 1.0)
            w_ml  = getattr(self, "w_ml",  1.0)
            w_ap  = getattr(self, "w_ap",  1.0)

            # Maximum actuation per DOF
            max_act_ins = np.max(np.abs(actuations[:, 0])) + 1e-6
            max_act_ml  = np.max(np.abs(actuations[:, 1])) + 1e-6
            max_act_ap  = np.max(np.abs(actuations[:, 2])) + 1e-6

            # Scale error by actuation / max_actuation
            scaled_e_ins = cp.multiply(e_ins, np.abs(actuations[:, 0]) / max_act_ins)
            scaled_e_ml  = cp.multiply(e_ml,  np.abs(actuations[:, 1]) / max_act_ml)
            scaled_e_ap  = cp.multiply(e_ap,  np.abs(actuations[:, 2]) / max_act_ap)


            # Weighted per-axis sums
            cost_ins = cp.sum(cp.multiply(rho, cp.square(scaled_e_ins)))
            cost_ml  = cp.sum(cp.multiply(rho, cp.square(scaled_e_ml)))
            cost_ap  = cp.sum(cp.multiply(rho, cp.square(scaled_e_ap)))

            # -------------------------------------------------------
            # 6. Final objective
            # -------------------------------------------------------
            objective = cp.Minimize(
                  cost_ins
                + cost_ml 
                + cost_ap
                + cp.norm(delta_j, 2)            # regularization on ΔJ
            )

            problem = cp.Problem(objective, [])

            # -------------------------------------------------------
            # 7. Solve optimization
            # -------------------------------------------------------
            problem.solve()

            iteration += 1

            if j_k1.value is not None:
                self.previous_jac = self.actual_jac
                self.actual_jac = j_k1.value
            else:
                print(f"Optimization failed at iteration {iteration}, status: {problem.status}")
                return 0

        # -------------------------------------------------------
        # Done
        # -------------------------------------------------------
        print(f"Optimization successful at iteration: {iteration}")
        print(f"Computed Jacobian: {self.actual_jac}")

        if self.store_previous:
            self.container.previous_jacobians.append(self.actual_jac)

        return 1

    
    def estimate_jacobian_decouple_ds(self):

        iteration = 0

        # -------------------------------------------------------
        # 1. Load latest experiment path
        # -------------------------------------------------------
        path_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", "experiments", "to_evaluate")
        )
        latest_file = os.path.join(path_dir, "LATEST")

        with open(latest_file, "r") as f:
            latest_experiment = f.read().strip()

        latest_experiment_path = os.path.join(latest_experiment, "logs")
        print(f"📁 Latest experiment path: {latest_experiment_path}")

        # -------------------------------------------------------
        #  Loop until convergence of J
        # -------------------------------------------------------
        while ((scipy.linalg.norm(self.actual_jac - self.previous_jac, "fro") > self.tolerance)
            and iteration < self.max_iter) or iteration < 1:

            # Save state if jump in Jacobian is excessive
            if scipy.linalg.norm(self.actual_jac - self.previous_jac, "fro") > 20:
                self.container.save_state(
                    directory=latest_experiment_path,
                    filename=f"state_{self.i}.json"
                )
                self.i += 1

            # -------------------------------------------------------
            # 2. Extract buffer contents
            # -------------------------------------------------------
            if self.container.is_filling:
                N = self.container.current_index
                indices = range(N)

                dp          = self.container.dp[:N]              # (N,3)
                actuations  = self.container.actuations[:N]      # (N,3)
                rho_values  = self.container.rho[:N, 0].copy()   # (N,)
                rho_values  = rho_values[::-1]                   # flip, preserves shape

            else:
                # Circular buffer case
                horizon = self.container.horizon_size
                indices = [(self.container.current_index + i) % horizon for i in range(horizon)]

                dp         = self.container.dp[indices]
                actuations = self.container.actuations[indices]

                rho_values = np.roll(self.container.rho[:, 0], -self.container.current_index)

            N = len(dp)

            # -------------------------------------------------------
            # 3. Update rho using forgetting + dead-zone normalization
            # -------------------------------------------------------
            updated_rho = np.zeros(N)

            for k, idx in enumerate(indices):
                ds = np.linalg.norm(self.container.dp[idx])
                updated_rho[k] = (
                    (self.container.lambda_forget ** k)
                    * (ds ** 2)
                    / (ds ** 2 + self.container.sigma_forget ** 2)
                )

            rho = updated_rho  # ensure clean 1-D vector shape

            # -------------------------------------------------------
            # 4. CVXPY variables
            # -------------------------------------------------------
            delta_j = cp.Variable(self.actual_jac.shape)
            j_k1 = self.actual_jac + delta_j

            predictions = (j_k1 @ dp.T).T          # (N,3)
            error_matrix = actuations - predictions

            e_ins = error_matrix[:, 0]
            e_ml  = error_matrix[:, 1]
            e_ap  = error_matrix[:, 2]

            # Directional weights
            w_ins = getattr(self, "w_ins", 1.0)
            w_ml  = getattr(self, "w_ml",  1.0)
            w_ap  = getattr(self, "w_ap",  1.0)

            # -------------------------------------------------------
            # 5. Costs
            # -------------------------------------------------------
            cost_ins = cp.sum(cp.multiply(rho, cp.square(e_ins)))
            cost_ml  = cp.sum(cp.multiply(rho, cp.square(e_ml)))
            cost_ap  = cp.sum(cp.multiply(rho, cp.square(e_ap)))

            reg = 0.001 * cp.sum_squares(delta_j)
            objective = cp.Minimize(
                w_ins * cost_ins +
                w_ml  * cost_ml +
                w_ap  * cost_ap +
                reg
            )


            problem = cp.Problem(objective)

            # -------------------------------------------------------
            # 6. Solve optimization
            # -------------------------------------------------------
            try:
                problem.solve(solver=cp.OSQP, verbose=False)
            except Exception as e:
                print("CVXPY failed:", e)
                return 0

            iteration += 1

            if j_k1.value is not None:
                self.previous_jac = self.actual_jac
                self.actual_jac = j_k1.value
            else:
                print(f"Optimization failed at iter {iteration}, status = {problem.status}")
                return 0

        # -------------------------------------------------------
        # Done
        # -------------------------------------------------------
        print(f"Optimization successful at iteration: {iteration}")
        print("Computed Jacobian:\n", self.actual_jac)

        if self.store_previous:
            self.container.previous_jacobians.append(self.actual_jac)

        return 1

    
    
    
    
    
    
    def estimate_jacobian(self):
      iteration = 0

      while (scipy.linalg.norm(self.actual_jac - self.previous_jac, 'fro') > self.tolerance and iteration < self.max_iter) or iteration < 1:

          # Estrai dati secondo il tipo di riempimento
          if self.container.is_filling:
              indices = range(self.container.current_index)
              dp = self.container.dp[:self.container.current_index]
              actuations = self.container.actuations[:self.container.current_index]
              rho = np.flip(self.container.rho[:self.container.current_index,0])
          else:
              horizon = self.container.horizon_size
              indices = [(self.container.current_index + i) % horizon for i in range(horizon)]
              dp = self.container.dp[indices]
              actuations = self.container.actuations[indices]
              rho = np.roll(self.container.rho, -self.container.current_index)
              rho = np.mean(rho,axis = 1)
          # Aggiorna errori e pesi rho
          
          for idx, i in enumerate(indices):
              error_vec = (
                  self.container.actuations[i]
                  - self.actual_jac @ self.container.dp[i]
                  - self.container.disturbance[i]
              )
              self.container.errors[i] = np.linalg.norm(error_vec, 2)
              self.container.rho[i] = (
                  (self.container.lambda_forget ** idx)
                  * (self.container.errors[i] ** 2)
                  / (self.container.errors[i] ** 2 + self.container.sigma_forget ** 2)
              )
          
          # Definisci variabili per l'ottimizzazione
          delta_j = cp.Variable(self.actual_jac.shape)
          j_k1 = self.actual_jac + delta_j
          error_matrix = actuations - (j_k1 @ dp.T).T  # (N, 3)
          
          
          # Calcolo della funzione obiettivo
          row_norms_squared = cp.sum(cp.square(error_matrix), axis=1)  # (N,)


          weighted_errors = cp.multiply(rho, row_norms_squared)
          weighted_sum = cp.sum(weighted_errors)

          objective = cp.Minimize(weighted_sum + cp.norm(delta_j, 2))
          problem = cp.Problem(objective, [])
          
          problem.solve()

          iteration += 1

          if j_k1.value is not None:
              self.previous_jac = self.actual_jac
              self.actual_jac = j_k1.value
          else:
              print(f"Optimization failed at iteration {iteration}, status: {problem.status}")
              return 0
      #print(f"Optimization successful at iteration {iteration}")
      if self.store_previous:
          self.container.previous_jacobians.append(self.actual_jac)

      return 1  # Successo
   
   
   