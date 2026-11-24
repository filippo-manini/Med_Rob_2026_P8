#!/usr/bin/env python3

import rospy as rp
import os
import csv
import time
import math
import numpy as np
import sympy as sp
import pandas as pd

from aurora_processing import AuroraProcessing
from arduino_communicator import ArduinoCommunicator


def compute_analytical_jacobian(x_meas, y_meas, z_meas, catheter_diameter=3.0):
    """
    Computes the analytical 3x3 Jacobian matrix evaluated at the measured position
    (x_meas, y_meas, z_meas) based on the Constant Curvature (CC) kinematic model.
    """
    x, y, z = sp.symbols('x y z')
    d = catheter_diameter  # catheter diameter (mm)

    phi = sp.atan2(y, x)
    k = 2 * sp.sqrt(x ** 2 + y ** 2) / (x ** 2 + y ** 2 + z ** 2)
    theta = sp.acos(1 - k * sp.sqrt(x ** 2 + y ** 2))
    l = theta / k  # arc length (mm)

    q_inv = sp.zeros(3, 1)
    q_inv[0] = l                                             # Insertion (INS)
    q_inv[1] = -d / 2 * theta * sp.cos(math.pi / 2 - phi)    # ML
    q_inv[2] = -d / 2 * theta * sp.cos(-phi)                 # AP

    J = sp.Matrix(q_inv).jacobian([x, y, z])

    if abs(x_meas) < 1e-6 and abs(y_meas) < 1e-6:
        x_meas = 1e-6

    J_numeric = J.subs({x: x_meas, y: y_meas, z: z_meas})
    return np.array(J_numeric).astype(np.float64)


def is_too_close(candidate_q, visited_list, radii):
    """
    Returns True if candidate_q falls inside the normalized exclusion radius
    of any previously visited point in visited_list.
    """
    if not visited_list:
        return False
    visited_arr = np.array(visited_list)
    normalized_dists = np.linalg.norm((visited_arr - candidate_q) / radii, axis=1)
    return np.any(normalized_dists < 1.0)


def compute_coverage_from_csv(csv_path, limits_min=(0, 0, -950), limits_max=(14000, 1400, 2050), grid_bins=(5, 4, 4), only_explore=False):
    df = pd.read_csv(csv_path)

    if only_explore and 'mode' in df.columns:
        df = df[df['mode'] == 'EXPLORE']

    if df.empty:
        return 0.0, 0, int(np.prod(grid_bins))

    cols = ['current_ins', 'current_ml', 'current_ap']
    voxel_df = pd.DataFrame()

    for col, l_min, l_max, n_bins in zip(cols, limits_min, limits_max, grid_bins):
        # Discretize each axis into equal-width bins
        voxel_df[col] = pd.cut(
            df[col],
            bins=np.linspace(l_min, l_max, n_bins + 1),
            labels=False,
            include_lowest=True
        )

    # Drop any out-of-bounds rows (NaN) and count unique 3D voxels
    unique_voxels = voxel_df.dropna().drop_duplicates()
    total_voxels = int(np.prod(grid_bins))
    coverage_pct = (len(unique_voxels) / total_voxels) * 100.0

    return coverage_pct, len(unique_voxels), total_voxels


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

    DURATION_SECONDS = 300.0  # Example: 5 minutes (300 seconds)

    with open(csv_file, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "timestamp",
            "elapsed_time_s",
            "mode",
            "p_start_x", "p_start_y", "p_start_z",
            "J11", "J12", "J13",
            "J21", "J22", "J23",
            "J31", "J32", "J33",
            "delta_ins", "delta_ml", "delta_ap",
            "p_reached_x", "p_reached_y", "p_reached_z",
            "current_ins", "current_ml", "current_ap"
        ])

        input(f"\n[READY] Press ENTER to start hybrid acquisition for {DURATION_SECONDS} s...")

        # Cumulative motor step tracker [INS, ML, AP]
        current_q = np.array([0, 0, 0], dtype=int)
        visited_q = []  # Only stores points from EXPLORE mode
        
        # Physical workspace limits in motor steps
        LIMITS_MIN = np.array([0, -400, -400])       # [INS_min, ML_min, AP_min]
        LIMITS_MAX = np.array([5000, 400, 400])      # [INS_max, ML_max, AP_max]
        
        # Exclusion radius per axis [INS, ML, AP] to force spatial variability
        EXCLUSION_RADII = np.array([150.0, 25.0, 25.0])
        MAX_RETRIES = 50

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

                if np.all(np.abs(diff) <= 15):
                    mode = "EXPLORE"
                    local_steps_left = np.random.randint(LOCAL_BURST_MIN, LOCAL_BURST_MAX + 1)
                    print(f"\n[MODE SWITCH] Reached target region {current_q}. Starting {local_steps_left} local moves...")
                    continue

                delta_q = np.clip(diff, -MAX_TRAVEL_STEP, MAX_TRAVEL_STEP)
                proposed_q = current_q + delta_q

            elif mode == "EXPLORE":
                if local_steps_left <= 0:
                    mode = "TRAVEL"
                    # Pick a new global target outside the exclusion radius of explored points
                    for _ in range(MAX_RETRIES):
                        candidate_target = np.random.randint(LIMITS_MIN, LIMITS_MAX + 1)
                        if not is_too_close(candidate_target, visited_q, EXCLUSION_RADII):
                            break
                    target_q = candidate_target
                    print(f"\n[MODE SWITCH] Local burst finished. Traveling to new global target: {target_q}...")
                    continue

                # Sample random local actuations within limits and outside exclusion radius
                found_valid = False
                for _ in range(MAX_RETRIES):
                    delta_q = np.array([
                        np.random.choice(INSERTION_RANGE),
                        np.random.choice(ML_AP_RANGE),
                        np.random.choice(ML_AP_RANGE)
                    ])
                    proposed_q = current_q + delta_q

                    if np.all(proposed_q >= LIMITS_MIN) and np.all(proposed_q <= LIMITS_MAX):
                        if not is_too_close(proposed_q, visited_q, EXCLUSION_RADII):
                            found_valid = True
                            break

                # Fallback if local area is saturated: accept any move within limits
                if not found_valid:
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
            if mode == "EXPLORE":
                visited_q.append(current_q.copy())

            # Brief mechanical stabilization pause (settle time)
            rp.sleep(0.5)

            # 2) SAMPLE P_bar (New reached tip pose in base_frame)
            p_bar = processor.tip_pose[:3, 3].copy()

            # Write full data row to CSV
            writer.writerow([
                rp.get_time(),
                round(elapsed, 3),
                mode,
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
    
    cov_all, visited_all, total = compute_coverage_from_csv(csv_file, only_explore=False)
    cov_exp, visited_exp, _ = compute_coverage_from_csv(csv_file, only_explore=True)
    print(f"[STATS] Workspace coverage (ALL):     {cov_all:.2f}% ({visited_all}/{total} regions)")
    print(f"[STATS] Workspace coverage (EXPLORE): {cov_exp:.2f}% ({visited_exp}/{total} regions)")
    
    print("[DONE] Resetting motors to home position...")
    communicator.send_reset_command()


if __name__ == '__main__':
    try:
        main()
    except rp.ROSInterruptException:
        pass