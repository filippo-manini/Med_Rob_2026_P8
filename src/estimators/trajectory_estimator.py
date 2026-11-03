#!/usr/bin/env python3

import numpy as np
import casadi as ca
import scipy.linalg
import cvxpy as cp
import sympy as sp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

import rospy as rp


import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
   
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

from scipy.optimize import minimize

class ConstantCurvatureFitter:
    def __init__(self):
        pass  # Eventualmente puoi passare parametri globali qui

    def cc_forward_kinematics_correct(self, k, theta, phi, l):
        # L'arc length è l = theta / k, quindi non serve scalare manualmente
        x = (1 / k) * np.cos(phi) * (1 - np.cos(theta))
        y = (1 / k) * np.sin(phi) * (1 - np.cos(theta))
        z = (1 / k) * np.sin(theta)
        return np.array([x, y, z])

    def generate_curve_endpoint(self, tip_pose, k, theta, phi, l):
        """
        Calcola la posizione finale della curva CC nel frame base.
        """
        R_tip = tip_pose[:3, :3]
        t_tip = tip_pose[:3, 3]

        local_endpoint = self.cc_forward_kinematics_correct(k, theta, phi, l)
        global_endpoint = R_tip @ local_endpoint + t_tip
        return global_endpoint

    def fit_cc_params(self, tip_pose, target, initial_guess=None):
        if initial_guess is None:
            initial_guess = np.array([0.05, 1.0, 0.0, 40.0])
        #print("initial guess: ", initial_guess)

        bounds = [
            (1e-6, 0.5),        # curvatura massima maggiore
            (0.0, 2 * np.pi),   # angolo fino a 360°
            (-np.pi, np.pi),    # phi completo
            (1.0, 200.0)        # lunghezza massima 200 mm
        ]

        def constraint_eq(params):
            k, theta, phi, l = params
            return self.generate_curve_endpoint(tip_pose, k, theta, phi, l) - target

        constraint = {'type': 'eq', 'fun': constraint_eq}

        def objective(params):
            k, theta, phi, l = params
            return k**2 + 0.001 * l  # penalizzazione su k e l

        res = minimize(objective, initial_guess, bounds=bounds, constraints=[constraint], method='SLSQP')

        if res.success or "Positive directional derivative" in res.message:
            k_opt, theta_opt, phi_opt, l_opt = res.x
            end_error = np.linalg.norm(self.generate_curve_endpoint(tip_pose, k_opt, theta_opt, phi_opt, l_opt) - target)
            print(f"Success! Endpoint error = {end_error:.6e}")
            return {
                'k': k_opt,
                'theta': theta_opt,
                'phi': phi_opt,
                'l': l_opt,
                'fun': res.fun
            }
        else:
            raise RuntimeError("Ottimizzazione fallita: " + res.message)








class TrajectoryEstimator:
    def __init__(self, dim, jacobian_estimator):

        self.dim = dim
        self.analytical = False
        self.jacobian_estimator = jacobian_estimator

        self.min_step = 0.1
        self.max_step = 10
        self.f_weight = 0.1
        self.actuator_weight = 30
        self.pos_weight = 1
        self.target_weight = 15
        self.tol = 1e-2
        self.max_iter = 500
        self.verbosity = 1
        self.opti = ca.Opti()
        self.maximal_insertable_length = 18600
        self.workspace_mapper = None

    def validate_and_finalize(self):
        self.tol = float(self.tol)
    
    
    def express_point_in_frame(self, actual_position, tip_target):
        
        #print("USING ACTUAL POSITION:", actual_position)
        #print("USING TIP POSITION:", tip_target)

        R = actual_position[:3, :3]   # rotazione
        t = actual_position[:3, 3]    # traslazione
        

  

        # Trasforma il punto nel frame attuale
        relative_point = R.T @ (tip_target - t)
        #print("RELATIVE POINT: ",relative_point)
        return relative_point
    
    def build_problem_classic_MPC(self, prediction_horizon, actual_position, target_position):
        """
        Builds the optimization problem for trajectory planning.

        Parameters:
        - prediction_horizon (int): Number of prediction steps (N)
        - actual_position (np.array): Current pose (4x4 matrix or [x,y,z])
        - target_position (list or np.array): Target position [x,y,z] or trajectory
        
        """
        # Use only final target point if a trajectory is passed
        if len(np.shape(target_position)) > 1:
            target_position = target_position[-1]

        N, dim = prediction_horizon, self.dim
        opti = self.opti
        # Extract current position
        if np.shape(actual_position) == (4, 4):
            self.x0 = np.array(actual_position[:3, 3]).reshape(dim)
        else:
            self.x0 = np.array(actual_position).reshape(dim)

        # Define optimization variable
        self.X = opti.variable(N, dim)

        # Target
        self.xN = np.array(target_position).reshape(dim)

        # Initial trajectory guess
        self.initial_guess = np.linspace(self.x0, self.xN, N)

        # Jacobian
        if self.analytical:
            self.jac_fun = self.jacobian_estimator.build_Inverse_jacobian_fun()
        else:
            self.jac_fun = ca.inv(ca.DM(self.jacobian_estimator.actual_jac))

        # Build cost
        # --- Cost function (CasADi-safe, column-based) ---

        xN_dm = ca.DM(self.xN).reshape((3, 1))
        starting_point = ca.DM(self.x0).reshape((3, 1))

        W_pos = ca.DM(self.pos_weight * np.ones(3)).reshape((3, 1))
        W_act = ca.DM(self.actuator_weight * np.ones(3)).reshape((3, 1))

        cost = 0
        for k in range(N - 1):

            u_k = self.X[k, :].T   # (3×1)

            if self.analytical:
                Jinv_k = self.jac_fun(starting_point)   # evaluate Function
            else:
                Jinv_k = self.jac_fun                   # constant DM

            dx_k = ca.mtimes(Jinv_k, u_k)
            next_point = starting_point + dx_k

            e_k = next_point - xN_dm
            cost += ca.sumsqr(ca.sqrt(W_pos) * e_k)
            cost += ca.sumsqr(ca.sqrt(W_act) * u_k)

            starting_point = next_point


        # Assign to optimization problem
        opti.minimize(cost)

        # Set initial guess
        opti.set_initial(self.X, ca.DM(self.initial_guess))

        # Solver configuration
        opti.solver('ipopt', {
            'ipopt': {
                'tol': self.tol,
                'max_iter': self.max_iter,
                'linear_solver': 'mumps',
                'print_level': self.verbosity,
                'hessian_approximation': 'limited-memory'
            }
        })

        #print("Optimization problem built successfully.")

            
               
            
        
        
        
        
        
        
        
        
        
        
        


    def build_problem(self, prediction_horizon, actual_position, target_position):
        """
        Builds the optimization problem for trajectory planning.

        Parameters:
        - prediction_horizon (int): Number of prediction steps (N)
        - actual_position (list or np.array): Current position [x, y, z]
        - target_position (list of list/array): List of target positions for each step, length must be N+1
        """
        
        # Check that target trajectory matches prediction horizon
        if len(target_position) != prediction_horizon + 1:
            raise ValueError("`target_position` must have length `prediction_horizon + 1`.")

        N, dim = prediction_horizon, self.dim
        opti = self.opti
        # Initial state
        self.x0 = np.array(actual_position[:3, 3]).reshape(1, dim)
        self.X = opti.variable(N + 1, dim)
        opti.subject_to(self.X[0, :] == self.x0)

        #print("Target trajectory:", target_position)

        # Final target point
        self.xN = np.array(target_position[0]).reshape(1, dim)
        #print("Final target position:", self.xN)

        
        
        
        
        points = np.linspace(self.x0, self.xN, N + 1).reshape(N + 1, dim)  
        #print("actual_position", actual_position)
        #print("Initial guess trajectory points:", points)
        

        self.initial_guess = points
        self.jac_fun = self.jacobian_estimator.build_jacobian_fun()

        # Cost function
        cost = 0
        task_cost = 0
        smoothness_cost = 0
        terminal_cost = 0
        xN_dm = ca.DM(self.xN)

        for k in range(N):
            delta = self.X[k + 1, :] - self.X[k, :]
            
            # Step-size constraint (optional, may remove or soften)
            # opti.subject_to(ca.norm_2(delta) >= self.min_step)

            # Jacobian (analytical or data-driven)
            if self.analytical:
                vars_vec_k = ca.vertcat(self.X[k, 0], self.X[k, 1], self.X[k, 2])
                J_k = self.jac_fun(vars_vec_k)[0]
                print( "USING ANALYTICAL \r\n")
            else:
                J_k = ca.DM(self.jacobian_estimator.actual_jac)

            # Forward kinematics (actuator movement)
            delta_col = ca.reshape(delta, 3, 1)
            f_k_k = ca.mtimes(J_k, delta_col)

            # New: add Cartesian tracking term
            
            target_kp1 = ca.reshape(ca.DM(target_position[k + 1]), 1, 3)  # ✅ Matches shape of self.X[k+1, :]

            task_cost += self.f_weight * ca.sumsqr(self.X[k + 1, :] - target_kp1)

            # Optional: actuator effort term (regularization)
            #task_cost += self.actuator_weight * ca.sumsqr(f_k_k)

            # Smoothness
            smoothness_cost += self.pos_weight * ca.sumsqr(delta)


        # Terminal cost (penalty on distance from final target)
        terminal_cost += self.target_weight * ca.sumsqr(self.X[N, :] - xN_dm)
        cost = task_cost + smoothness_cost + terminal_cost
        self.task_cost_fun = ca.Function("task_cost", [self.X], [task_cost])
        self.smoothness_cost_fun = ca.Function("smoothness_cost", [self.X], [smoothness_cost])
        self.terminal_cost_fun = ca.Function("terminal_cost", [self.X], [terminal_cost])
        # Set cost function in optimizer
        opti.minimize(cost)

        # Set initial guess
        opti.set_initial(self.X, ca.DM(self.initial_guess))

        # Solver configuration
        opti.solver('ipopt', {
            'ipopt': {
                'tol': self.tol,
                'max_iter': self.max_iter,
                'linear_solver': 'mumps',
                'print_level': self.verbosity,
                'hessian_approximation': 'limited-memory'
            }
        })

        #print("Optimization problem built successfully.")


    def build_problem_dynamical_weight(self, prediction_horizon, actual_position, target_position, len_inserted):
        """
        Build the MPC optimization problem with dynamic weighting on actuation directions.

        Parameters:
            prediction_horizon (int): Number of prediction steps N.
            actual_position (4x4 SE3 matrix or 3-vector): Current robot tip pose or position.
            target_position (list of length N+1): Reference Cartesian positions (world frame).
            len_inserted (float): Current insertion length.
        """

        # -------------------------------
        # 1. Input Validation
        # -------------------------------
        N = prediction_horizon
        dim = self.dim
        opti = self.opti

        if len(target_position) != N + 1:
            raise ValueError("target_position must have length prediction_horizon + 1")

        if dim != 3:
            raise ValueError("This implementation currently supports dim == 3 only.")

        # -------------------------------
        # 2. Extract current position
        # -------------------------------
        actual_arr = np.array(actual_position)
        if actual_arr.shape == (4, 4):
            actual_pos = actual_arr[:3, 3]
        else:
            actual_pos = actual_arr.reshape(3)

        x0 = actual_pos.reshape(1, 3)
        self.x0 = x0

        # -------------------------------
        # 3. Define optimization variables
        # -------------------------------
        X = opti.variable(N + 1, dim)
        self.X = X

        # Initial condition constraint
        opti.subject_to(X[0, :] == x0)

        # -------------------------------
        # 4. Final target point
        # -------------------------------
        xN = np.array(target_position[0]).reshape(1, 3)
        self.xN = xN
        xN_dm = ca.DM(xN)

        # -------------------------------
        # 5. Initial trajectory guess
        # -------------------------------
        initial_guess = np.linspace(x0, xN, N + 1).reshape(N + 1, 3)
        self.initial_guess = initial_guess
        opti.set_initial(X, ca.DM(initial_guess))

        # -------------------------------
        # 6. Jacobian function
        # -------------------------------
        
        self.jac_fun = self.jacobian_estimator.build_jacobian_fun()

        # -------------------------------
        # 7. Dynamic weighting α(len)
        # -------------------------------
        norm_len = np.clip(len_inserted / self.maximal_insertable_length, 0.0, 1.0)

        steepness = 30
        midpoint = (self.maximal_insertable_length - 6000) / self.maximal_insertable_length
        alpha = 1 / (1 + ca.exp(-steepness * (norm_len - midpoint)))

        # Define early/late weights
        w_ins_early = 0.01 * self.actuator_weight    # encourage insertion early
        w_ins_late  = 1.0  * self.actuator_weight    # normal insertion penalty

        w_lat_early = 10.0  * self.actuator_weight    # penalize ML/AP early
        w_lat_late  = 1.0  * self.actuator_weight    # normal lateral penalty

        # Blend with α
        insertion_w = (1-alpha)*w_ins_early + alpha*w_ins_late
        ml_w        = (1-alpha)*w_lat_early + alpha*w_lat_late
        ap_w        = (1-alpha)*w_lat_early + alpha*w_lat_late

        W = ca.diag(ca.vertcat(insertion_w, ml_w, ap_w))

        # -------------------------------
        # 8. Cost accumulators
        # -------------------------------
        task_cost = 0
        smoothness_cost = 0
        actuation_cost = 0

        # -------------------------------
        # 9. Build stage costs
        # -------------------------------
        for k in range(N):

            delta = X[k + 1, :] - X[k, :]

            # Jacobian at X[k]
            if self.analytical:
                J_k = self.jac_fun(X[k, :].T)[0]
                print("using analytical")
            else:
                J_k = ca.DM(self.jacobian_estimator.actual_jac)

            # Compute actuation (virtual input)
            delta_col = ca.reshape(delta, 3, 1)
            f_k = ca.mtimes(J_k, delta_col)

            # Target for next step
            target_kp1 = ca.reshape(ca.DM(target_position[k + 1]), 1, 3)

            # Tracking cost
            task_cost += self.f_weight * ca.sumsqr(X[k + 1, :] - target_kp1)

            # Actuation usage penalty
            actuation_cost += ca.mtimes([f_k.T, W, f_k])

            # Smoothness cost
            smoothness_cost += self.pos_weight * ca.sumsqr(delta)

        # -------------------------------
        # 10. Terminal cost
        # -------------------------------
        
        terminal_cost = self.target_weight * ca.sumsqr(X[N, :] - xN_dm)

        # -------------------------------
        # 11. Total cost
        # -------------------------------
        total_cost = task_cost + smoothness_cost + terminal_cost
        # (IMPORTANT FIX: include actuation_cost)
        total_cost += actuation_cost

        # Store cost functions
        self.task_cost_fun = ca.Function("task_cost", [X], [task_cost])
        self.smoothness_cost_fun = ca.Function("smoothness_cost", [X], [smoothness_cost])
        self.terminal_cost_fun = ca.Function("terminal_cost", [X], [terminal_cost])
        self.actuation_cost_fun = ca.Function("actuation_cost", [X], [actuation_cost])

        opti.minimize(total_cost)

        # -------------------------------
        # 12. Solver options
        # -------------------------------
        opti.solver(
            'ipopt',
            {
                'ipopt': {
                    'tol': self.tol,
                    'max_iter': self.max_iter,
                    'linear_solver': 'mumps',
                    'print_level': self.verbosity,
                    'hessian_approximation': 'limited-memory',
                }
            }
        )

        
        
    def build_problem_dynamical_weight_constraints(self, prediction_horizon, actual_position, target_position, len_inserted):
        """
        Build the MPC optimization problem with dynamic weighted actuation penalties
        AND workspace constraints (ML, AP, Insertion tendon ranges).

        The states are:
            X[k] = Cartesian tip position (1x3)
            U[k] = tendon positions [ML, AP, INS] (1x3)

        Constraints enforced:
            max_M <= U[k,0] <= max_L          (ML workspace)
            max_P <= U[k,1] <= max_A          (AP workspace)
            0     <= U[k,2] <= Max_INS        (Insertion)
        """

        N = prediction_horizon
        dim = self.dim
        opti = self.opti

        # -----------------------------------------
        # 0. Input validation
        # -----------------------------------------
        if len(target_position) != N + 1:
            raise ValueError("target_position must have length prediction_horizon+1")
        if dim != 3:
            raise ValueError("This implementation currently supports only 3D state dim.")

        # -----------------------------------------
        # 1. Extract current Cartesian position
        # -----------------------------------------
        actual_pos = (
            actual_position[:3, 3]
            if np.array(actual_position).shape == (4, 4)
            else np.array(actual_position).reshape(3)
        )
        x0 = actual_pos.reshape(1, 3)
        self.x0 = x0

        # -----------------------------------------
        # 2. Optimization variables
        # -----------------------------------------
        # State trajectory in Cartesian space
        X = opti.variable(N + 1, 3)
        self.X = X

        # Tendon trajectory: U[k] = [ML, AP, INS]
        U = opti.variable(N + 1, 3)
        self.U = U

        # Set initial x and U
        opti.subject_to(X[0, :] == x0)

        U0 = ca.DM([
            self.workspace_mapper.actual_ML_actuation,
            self.workspace_mapper.actual_AP_actuation,
            self.workspace_mapper.actual_insertion
        ]).T
        opti.subject_to(U[0, :] == U0)

        # -----------------------------------------
        # 3. Final target point
        # -----------------------------------------
        xN = np.array(target_position[0]).reshape(1, 3)
        self.xN = xN
        xN_dm = ca.DM(xN)

        # -----------------------------------------
        # 4. Initial guess for X
        # -----------------------------------------
        initial_guess = np.linspace(x0, xN, N + 1).reshape(N + 1, 3)
        self.initial_guess = initial_guess
        opti.set_initial(X, ca.DM(initial_guess))

        # -----------------------------------------
        # 5. Jacobian function
        # -----------------------------------------
        self.jac_fun = self.jacobian_estimator.build_jacobian_fun()

        # -----------------------------------------
        # 6. Dynamic direction weighting using alpha
        # -----------------------------------------
        norm_len = np.clip(len_inserted / self.maximal_insertable_length, 0.0, 1.0)
        steepness = 30
        midpoint = 0.75

        alpha = 1 / (1 + ca.exp(-steepness * (norm_len - midpoint)))

        insertion_w = self.actuator_weight * (1 - alpha)
        ml_w        = self.actuator_weight * alpha
        ap_w        = self.actuator_weight * alpha

        W = ca.diag(ca.vertcat(ml_w, ap_w, insertion_w))

        # -----------------------------------------
        # 7. Cost terms
        # -----------------------------------------
        task_cost = 0
        smoothness_cost = 0
        actuation_cost = 0

        # -----------------------------------------
        # 8. Loop over horizon: dynamics + constraints + cost
        # -----------------------------------------
        for k in range(N):

            # ---------------------
            # POSITION DYNAMICS
            # ---------------------
            delta = X[k + 1, :] - X[k, :]

            # Jacobian
            if self.analytical:
                J_k = self.jac_fun(X[k, :].T)[0]
            else:
                J_k = ca.DM(self.jacobian_estimator.actual_jac)

            delta_col = ca.reshape(delta, 3, 1)
            f_k = ca.mtimes(J_k, delta_col)   # actuations (ML, AP, INS) increments

            # ---------------------
            # TENDON DYNAMICS
            # ---------------------
            opti.subject_to(U[k + 1, 0] == U[k, 0] + f_k[0])  # ML
            opti.subject_to(U[k + 1, 1] == U[k, 1] + f_k[1])  # AP
            opti.subject_to(U[k + 1, 2] == U[k, 2] + f_k[2])  # INS

            # ---------------------
            # WORKSPACE CONSTRAINTS
            # ---------------------
            # ML range
            opti.subject_to(U[k + 1, 0] >= self.workspace_mapper.max_M)
            opti.subject_to(U[k + 1, 0] <= self.workspace_mapper.max_L)

            # AP range
            opti.subject_to(U[k + 1, 1] >= self.workspace_mapper.max_P)
            opti.subject_to(U[k + 1, 1] <= self.workspace_mapper.max_A)

            # Insertion
            opti.subject_to(U[k + 1, 2] >= 0)
            opti.subject_to(U[k + 1, 2] <= self.workspace_mapper.Max_INS)

            # ---------------------
            # COST TERMS
            # ---------------------
            # Tracking
            target_kp1 = ca.reshape(ca.DM(target_position[k + 1]), 1, 3)
            task_cost += self.f_weight * ca.sumsqr(X[k + 1, :] - target_kp1)

            # Penalize actuation directions
            actuation_cost += ca.mtimes([f_k.T, W, f_k])

            # Smoothness
            smoothness_cost += self.pos_weight * ca.sumsqr(delta)

        # -----------------------------------------
        # 9. Terminal Cost
        # -----------------------------------------
        term_weight = (1 + (1 - alpha) * 5) * self.target_weight
        terminal_cost = term_weight * ca.sumsqr(X[N, :] - xN_dm)

        total_cost = task_cost + smoothness_cost + terminal_cost + actuation_cost

        # Save cost reporters
        self.task_cost_fun = ca.Function("task_cost", [X], [task_cost])
        self.smoothness_cost_fun = ca.Function("smoothness_cost", [X], [smoothness_cost])
        self.terminal_cost_fun = ca.Function("terminal_cost", [X], [terminal_cost])
        self.actuation_cost_fun = ca.Function("actuation_cost", [X], [actuation_cost])

        # -----------------------------------------
        # 10. Set objective and solver
        # -----------------------------------------
        opti.minimize(total_cost)

        opti.solver(
            'ipopt',
            {
                'ipopt': {
                    'tol': self.tol,
                    'max_iter': self.max_iter,
                    'linear_solver': 'mumps',
                    'print_level': self.verbosity,
                    'hessian_approximation': 'limited-memory'
                }
            }
        )


    def report_cost_breakdown(self):
        X_sol = self.X_sol  # from solved problem

        task_val = float(self.task_cost_fun(X_sol))
        smooth_val = float(self.smoothness_cost_fun(X_sol))
        term_val = float(self.terminal_cost_fun(X_sol))
        act_val = float(self.actuation_cost_fun(X_sol))
        total = task_val + smooth_val + term_val + act_val

        print("=== Cost Breakdown ===")
        print(f"Task-space cost     : {task_val:.4f} ({task_val/total*100:.2f}%)")
        print(f"Smoothness cost     : {smooth_val:.4f} ({smooth_val/total*100:.2f}%)")
        print(f"Terminal cost       : {term_val:.4f} ({term_val/total*100:.2f}%)")
        print(f"actuation cost       : {act_val:.4f} ({act_val/total*100:.2f}%)")
        print(f"Total cost          : {total:.4f}")

    
    def solve(self):
        print("Solving optimization...")
        sol = self.opti.solve()
        self.X_sol = sol.value(self.X)
        #self.report_cost_breakdown()
        return self.X_sol

    def plot(self):
        if not hasattr(self, 'X_sol'):
            raise RuntimeError("Call .solve() before plotting.")

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')



        ax.plot(self.X_sol[:, 0], self.X_sol[:, 1], self.X_sol[:, 2], label='Traiettoria ottimale', marker='o')
        ax.scatter(self.xN[:, 0], self.xN[:, 1], self.xN[:, 2], color='r', label='Target', alpha=0.5)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # Trova i limiti globali per ogni asse, usando dati X_sol e xN
        all_points = np.vstack([self.X_sol, self.xN])
        xmin, ymin, zmin = np.min(all_points, axis=0)
        xmax, ymax, zmax = np.max(all_points, axis=0)

        # Calcola range massimo fra tutti gli assi per rendere le scale uguali
        max_range = max(xmax - xmin, ymax - ymin, zmax - zmin)

        # Centro degli assi
        xmid = (xmax + xmin) / 2
        ymid = (ymax + ymin) / 2
        zmid = (zmax + zmin) / 2

        # Imposta limiti con la stessa ampiezza per tutti e 3 gli assi
        ax.set_xlim(xmid - max_range/2, xmid + max_range/2)
        ax.set_ylim(ymid - max_range/2, ymid + max_range/2)
        ax.set_zlim(zmid - max_range/2, zmid + max_range/2)
        
        ax.legend()
        plt.show()
        
        
    def publish_trajectory(self):
        if not hasattr(self, 'path_pub'):
            self.path_pub = rp.Publisher('/trajectory_path', Path, queue_size=10)

        if self.initial_guess is None or len(self.initial_guess) == 0:
            rp.logwarn("No trajectory to publish")
            return

        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = rp.Time.now()
        path_msg.header.frame_id = "world"  # o "base_link", in base al tuo TF

        for point in self.initial_guess.copy():
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0] / 10
            pose.pose.position.y = point[1] / 10
            pose.pose.position.z = point[2] / 10

            # Orientazione neutra
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        
    