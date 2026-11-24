#!/usr/bin/env python3

import rospy as rp
import os
import csv
import time
import math
import numpy as np
import sympy as sp

from aurora_processing import AuroraProcessing
from arduino_communicator import ArduinoCommunicator


def compute_analytical_jacobian(x_meas, y_meas, z_meas, catheter_diameter=3.0):
    """
    Computes the analytical 3x3 Jacobian matrix evaluated at the measured position
    (x_meas, y_meas, z_meas) based on the Constant Curvature (CC) kinematic model.
    """
    # Define symbolic variables
    x, y, z = sp.symbols('x y z')
    d = catheter_diameter  # catheter diameter (mm)

    # Compute arc parameters
    phi = sp.atan2(y, x)
    k = 2 * sp.sqrt(x ** 2 + y ** 2) / (x ** 2 + y ** 2 + z ** 2)
    theta = sp.acos(1 - k * sp.sqrt(x ** 2 + y ** 2))
    l = theta / k  # arc length (mm)

    # Compute tendon displacements (inverse kinematics)
    q_inv = sp.zeros(3, 1)
    q_inv[0] = l                                             # Insertion (INS)
    q_inv[1] = -d / 2 * theta * sp.cos(math.pi / 2 - phi)    # ML
    q_inv[2] = -d / 2 * theta * sp.cos(-phi)                 # AP

    # Compute symbolic Jacobian matrix
    J = sp.Matrix(q_inv).jacobian([x, y, z])

    # Avoid exact singularity when x=0 and y=0 (k=0 -> division by zero in l = theta/k)
    if abs(x_meas) < 1e-6 and abs(y_meas) < 1e-6:
        x_meas = 1e-6

    # Substitute current tip position into the Jacobian
    J_numeric = J.subs({x: x_meas, y: y_meas, z: z_meas})

    return np.array(J_numeric).astype(np.float64)


def main():
    # =========================================================================
    # 1. INITIALIZE ROS NODES & MODULES
    # =========================================================================
    #rp.init_node('timed_random_dataset_acquisition', anonymous=True)
    
    communicator = ArduinoCommunicator()
    processor = AuroraProcessing(calibrated=False)

    # =========================================================================
    # 2. CALIBRATE BASE (IF IT IS FIRST TIME)
    # =========================================================================
    if not processor.flag_calibration_done:
        print("[ERROR] Base calibration was not completed.")
        return

    # =========================================================================
    # 3. CREATE CSV
    # =========================================================================
    log_dir = os.path.join(os.path.dirname(__file__), 'data_logs')
    os.makedirs(log_dir, exist_ok=True)
    csv_file = os.path.join(log_dir, f"nn_timed_dataset_{time.strftime('%Y%m%d_%H%M%S')}.csv")

    # Set total acquisition duration in seconds (X time)
    DURATION_SECONDS = 300.0  # Example: 5 minutes (300 seconds)

    with open(csv_file, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp",
            "elapsed_time_s",
            # Starting pose P_hat (3)
            "p_start_x", "p_start_y", "p_start_z",
            # Jacobian J(P_hat) (9)
            "J11", "J12", "J13",
            "J21", "J22", "J23",
            "J31", "J32", "J33",
            # Random actuation deltas (3)
            "delta_ins", "delta_ml", "delta_ap",
            # Reached pose P_bar (3)
            "p_reached_x", "p_reached_y", "p_reached_z",
            "current_ins", "current_ml", "current_ap"
        ])

        input(f"\n[READY] Press ENTER to start hybrid acquisition for {DURATION_SECONDS} s...")

        # Cumulative motor step tracker [INS, ML, AP]
        current_q = np.array([0, 0, 0], dtype=int)
        
        # Physical workspace limits in motor steps
        LIMITS_MIN = np.array([0, -400, -400])       # [INS_min, ML_min, AP_min]
        LIMITS_MAX = np.array([5000, 400, 400])      # [INS_max, ML_max, AP_max]
        
        # Maximum step increment per iteration during global travel
        MAX_TRAVEL_STEP = np.array([400, 40, 40])

        # Define local exploration ranges (excluding dead-zone steps)
        INSERTION_RANGE = np.concatenate([
            np.arange(-400, -199),
            np.arange(200, 401)
        ])

        ML_AP_RANGE = np.concatenate([
            np.arange(-40, -19),
            np.arange(20, 41)
        ])

        # Hybrid state machine variables ("TRAVEL" -> "EXPLORE")
        mode = "TRAVEL"
        target_q = np.random.randint(LIMITS_MIN, LIMITS_MAX + 1)
        local_steps_left = 0
        LOCAL_BURST_MIN = 10
        LOCAL_BURST_MAX = 20
        
        start_time = time.time()
        valid_samples = 0

        # =====================================================================
        # 4. START HYBRID ACQUISITION LOOP (GOVERNED BY TIME X)
        # =====================================================================
        while (time.time() - start_time) < DURATION_SECONDS and not rp.is_shutdown():
            
            # Wait until a valid Aurora tip pose is available
            if processor.tip_pose is None:
                rp.sleep(0.05)
                continue

            # 1) ACQUIRE STARTING POSE (P_hat)
            p_hat = processor.tip_pose[:3, 3].copy()

            # 2) COMPUTE J(P_hat) using the symbolic analytical Jacobian function
            J_hat = compute_analytical_jacobian(p_hat[0], p_hat[1], p_hat[2], catheter_diameter=3.0)

            # 3) SELECT ACTUATION ACCORDING TO CURRENT MODE
            if mode == "TRAVEL":
                diff = target_q - current_q

                # Check if the robot has reached the vicinity of the global target
                if np.all(np.abs(diff) <= 15):
                    mode = "EXPLORE"
                    local_steps_left = np.random.randint(LOCAL_BURST_MIN, LOCAL_BURST_MAX + 1)
                    print(f"\n[MODE SWITCH] Reached target region {current_q}. Starting {local_steps_left} local moves...")
                    continue

                # Move toward target_q in bounded steps
                delta_q = np.clip(diff, -MAX_TRAVEL_STEP, MAX_TRAVEL_STEP)
                proposed_q = current_q + delta_q

            elif mode == "EXPLORE":
                if local_steps_left <= 0:
                    # Finished local burst -> pick a new distant target anywhere in the workspace
                    mode = "TRAVEL"
                    target_q = np.random.randint(LIMITS_MIN, LIMITS_MAX + 1)
                    print(f"\n[MODE SWITCH] Local burst finished. Traveling to new global target: {target_q}...")
                    continue

                # Sample random local actuations from your dead-zone-free ranges within limits
                # This loop ensures that the proposed_q remains within the defined workspace limits
                while not rp.is_shutdown():
                    delta_q = np.array([
                        np.random.choice(INSERTION_RANGE),
                        np.random.choice(ML_AP_RANGE),
                        np.random.choice(ML_AP_RANGE)
                    ])
                    proposed_q = current_q + delta_q

                    if np.all(proposed_q >= LIMITS_MIN) and np.all(proposed_q <= LIMITS_MAX):
                        break

                local_steps_left -= 1

            # =================================================================
            # ACTUATIONS ARE IN LIMITS -> EXECUTE MOTION & SAMPLE P_bar
            # =================================================================
            valid_samples += 1
            elapsed = time.time() - start_time
            remaining = max(0.0, DURATION_SECONDS - elapsed)
            print(f"[{elapsed:.1f}s / {DURATION_SECONDS:.0f}s | Remaining: {remaining:.1f}s | {mode}] Sample #{valid_samples} -> delta_q: {delta_q}")

            # 1) MOVE ROBOT
            communicator.send_steps(delta_q)
            current_q = proposed_q.copy()

            # Brief mechanical stabilization pause (settle time)
            rp.sleep(0.5)

            # 2) SAMPLE P_bar (New reached tip pose in base_frame)
            p_bar = processor.tip_pose[:3, 3].copy()

            # Write full data row to CSV
            writer.writerow([
                rp.get_time(),
                round(elapsed, 3),
                p_hat[0], p_hat[1], p_hat[2],
                J_hat[0, 0], J_hat[0, 1], J_hat[0, 2],
                J_hat[1, 0], J_hat[1, 1], J_hat[1, 2],
                J_hat[2, 0], J_hat[2, 1], J_hat[2, 2],
                delta_q[0], delta_q[1], delta_q[2],
                p_bar[0], p_bar[1], p_bar[2],
                current_q[0], current_q[1], current_q[2]
            ])
            f.flush()

    print(f"\n[DONE] Time elapsed ({DURATION_SECONDS}s). Total valid samples collected: {valid_samples}.")
    print("[DONE] Resetting motors to home position...")
    communicator.send_reset_command()


if __name__ == '__main__':
    try:
        main()
    except rp.ROSInterruptException:
        pass