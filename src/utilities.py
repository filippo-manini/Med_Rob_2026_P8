from pathlib import Path
from datetime import datetime
import json, getpass, csv, logging
import rospy as rp
import os
import yaml
import shutil
import rosnode
import rosgraph  
import sys, termios, tty, threading
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np



def create_run_workspace(run_name=None, subdirs=None, config=None):
    """
    Create a timestamped run folder inside 'experiments' at the project root.
    Returns a dictionary with all important paths.
    """

    # Determine project root based on script location
    project_root = Path(__file__).resolve().parent.parent  # __file__ = script location
    experiments_dir = project_root / "experiments/to_evaluate"
    experiments_dir.mkdir(parents=True, exist_ok=True)  # create experiments if missing

    # Define subdirectories if not provided
    if subdirs is None:
        subdirs = ["logs", "results", "plots", "config"]

    # Create timestamped run folder if run_name not given
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    run_name = run_name or f"run_{timestamp}"
    run_dir = experiments_dir / run_name
    run_dir.mkdir(parents=True, exist_ok=True)

    # Create subfolders
    for sd in subdirs:
        (run_dir / sd).mkdir(parents=True, exist_ok=True)

    # Create README and meta.json
    (run_dir / "README.txt").write_text(
        "Aurora simulation run directory.\nFolders: logs/, results/, plots/, config/\n"
    )
    meta = {
        "created": timestamp,
        "user": getpass.getuser(),
        "run_name": run_name,
        "run_dir": str(run_dir),
        "subdirs": subdirs
    }
    
    config_path = config
    # Save the config into another directory using Pathlib
    output_path = Path(run_dir) / "config" / "config.yaml"
    shutil.copyfile(config_path, output_path)

    # Create empty results CSV
    results_csv = run_dir / "results" / "results.csv"
    if not results_csv.exists():
        fieldnames = ["time", "timestamp", "base_pos_x", "base_pos_y", "base_pos_z",
                      "tip_pos_x", "tip_pos_y", "tip_pos_z",
                      "target_tip_pos_x", "target_tip_pos_y", "target_tip_pos_z",
                      "valve_pos_x", "valve_pos_y", "valve_pos_z", 
                      "ACTUATION_INS", "ACTUATION_ML", "ACTUATION_AP",
                      "reached_tip_pos_x", "reached_tip_pos_y", "reached_tip_pos_z",
                      "position_error", "normalized_position_error", "target_error", "min_target_error"]

        with results_csv.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
    
    
    csv_logs = run_dir / "results" / "position_logs.csv"
    if not csv_logs.exists():
        fieldnames_logs = ["time", "timestamp", "base_pos_x", "base_pos_y", "base_pos_z",
                      "tip_pos_x", "tip_pos_y", "tip_pos_z",
                      "tip_pos_qx", "tip_pos_qy", "tip_pos_qz", "tip_pos_qw",
                      "valve_pos_x", "valve_pos_y", "valve_pos_z", 
                       "target_error", "min_target_error"]
        with csv_logs.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames_logs)
            writer.writeheader()
    
    
    # Create empty log file
    log_file = run_dir / "logs" / "run.log"
    log_file.touch()

    # Latest pointer
    (experiments_dir / "LATEST").write_text(str(run_dir))

    # Return all important paths
    return {
        "experiments_dir": str(experiments_dir),
        "run_dir": str(run_dir),
        "logs": str(run_dir / "logs"),
        "results": str(run_dir / "results"),
        "plots": str(run_dir / "plots"),
        "config": str(run_dir / "config"),
        "log_file": str(log_file),
        "results_csv": str(results_csv),
        "csv_logs": str(csv_logs),
        "fieldnames_dict": fieldnames,
        "fieldnames_logs_dict": fieldnames_logs
    }




def init_logging(log_file_path):
    """
    Initialize Python logging to both console and file.
    log_file_path: full path to logs/run.log
    """
    # Remove any existing handlers
    for handler in logging.root.handlers[:]:
        logging.root.removeHandler(handler)

    # Create handlers
    file_handler = logging.FileHandler(log_file_path)
    console_handler = logging.StreamHandler()

    # Define log format
    formatter = logging.Formatter('%(asctime)s [%(levelname)s]: %(message)s',
                                  datefmt='%Y-%m-%d %H:%M:%S')
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)

    # Configure root logger
    logging.basicConfig(level=logging.INFO, handlers=[file_handler, console_handler])

    logging.info("Logging initialized. All messages will go to console and %s", log_file_path)

class CSVPlotter:
    def __init__(self):
        self.processor = None
        self.communicator = None
        self.stop_requested = False
        self._finalized = False
        self._finalize_lock = threading.Lock()
        threading.Thread(target=self._keyboard_listener, daemon=True).start()

    def _keyboard_listener(self):
        rp.loginfo("Press 't' to stop and generate plots.")
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while True:
                ch = sys.stdin.read(1)
                if ch.lower() == "t":
                    self.stop_requested = True  # just set flag; control loop will finalize
                    rp.loginfo("Stop requested (t). Waiting for control loop to exit...")
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    # Run AFTER the loop exits
    def finalize(self):
        with self._finalize_lock:
            if self._finalized:
                return
            self._finalized = True

        rp.loginfo("Generating plots and rewinding robot...")
        df = None
        try:
            
            rp.loginfo(self.processor.run_paths["results_csv"])
            df = pd.read_csv(self.processor.run_paths["results_csv"])

            plt.figure()
            plt.plot(df["time"], df["position_error"], label="Position Error")
            plt.plot(df["time"], df["normalized_position_error"], label="Normalized Position Error")
            plt.plot(df["time"], df["target_error"], label="Target Error")
            plt.plot(df["time"], df["min_target_error"], label="Minimum Target Error")
            plt.plot()

            # Trova il minimo del position_error
            min_idx = df["target_error"].idxmin()
            min_time = df["time"][min_idx]
            min_value = df["target_error"][min_idx]

            # Aggiungi un marker nel punto del minimo
            plt.plot(min_time, self.processor.min_target_error, 'ro', label=f"Min Position Error ({self.processor.min_target_error:.3f} mm)")

            # (Opzionale) aggiungi un testo per evidenziare il valore
            plt.annotate(f"{self.processor.min_target_error:.3f} mm",
                        xy=(min_time, self.processor.min_target_error),
                        xytext=(min_time, self.processor.min_target_error + 0.5),  # sposta leggermente il testo in alto
                        arrowprops=dict(arrowstyle="->", color='red'),
                        fontsize=9,
                        color='red')

            plt.xlabel("Time [s]")
            plt.ylabel("Error norm [mm]")
            plt.legend()
            plt.grid(True)

            out_png = os.path.join(self.processor.run_paths["plots"], "errors.png")
            plt.savefig(out_png, dpi=150, bbox_inches="tight")
            plt.close()  # libera la figura

            
            
            deltas = []
            prev = None
            for jac in self.processor.controller.container.previous_jacobians:
                if prev is not None:
                    # Frobenius norm difference between consecutive Jacobians
                    delta = np.linalg.norm(jac - prev)
                    deltas.append(delta)
                prev = jac

            # Plot evolution of Jacobian differences
            plt.figure()
            plt.plot(df["time"].iloc[1:], deltas, marker="o")
            plt.xlabel("Step")
            plt.ylabel("Jacobian Δ (Frobenius norm)")
            plt.title("Change in Jacobian over time")
            out_png = os.path.join(self.processor.run_paths["plots"], "jacobian_deltas.png")
            plt.savefig(out_png, dpi=150, bbox_inches="tight")
            plt.close()

                                
                
        except Exception as e:
            rp.logerr(f"Failed to generate plots: {e}")
            df = None

        # bring the robot back after actuations
        try:
            if df is not None:
                cols = ["ACTUATION_INS", "ACTUATION_ML", "ACTUATION_AP"]
                if all(c in df.columns for c in cols):
                    rp.loginfo("Rewinding robot actuations...")
                    for _, row in df.iloc[::-1].iterrows():
                        delta = [-int(row[c]) for c in cols]
                        self.communicator.send_steps_to_arduino_go_back(delta)
                    rp.loginfo("Robot successfully brought back to start position.")
                else:
                    rp.logwarn("Actuation columns not found in CSV, skipping rewind.")
        except Exception as e:
            rp.logerr(f"Failed to rewind robot: {e}")

        # ROS-friendly shutdown
        try:
            rp.signal_shutdown("Stop requested and finalize completed.")
        except Exception:
            pass


        
    def finalize_one_shot(self, workspace_mapper):
        with self._finalize_lock:
            if self._finalized:
                return
            self._finalized = True

        rp.loginfo("Generating plots and rewinding robot...")
        df = None

        # -------------------- PLOTTING --------------------
        try:
            rp.loginfo(self.processor.run_paths["results_csv"])
            df = pd.read_csv(self.processor.run_paths["results_csv"])

            # ---- ERROR PLOT ----
            plt.figure()
            plt.plot(df["time"], df["position_error"], label="Position Error")
            plt.plot(df["time"], df["normalized_position_error"], label="Normalized Position Error")
            plt.plot(df["time"], df["target_error"], label="Target Error")
            plt.plot(df["time"], df["min_target_error"], label="Minimum Target Error")

            min_idx = df["min_target_error"].idxmin()
            min_time = df["time"][min_idx]
            min_value = df["min_target_error"][min_idx]

            plt.plot(min_time, min_value, 'ro', label=f"Min Target Error ({min_value:.3f} mm)")
            plt.annotate(f"{min_value:.3f} mm",
                        xy=(min_time, min_value),
                        xytext=(min_time, min_value + 0.5),
                        arrowprops=dict(arrowstyle="->", color='red'),
                        fontsize=9, color='red')

            plt.xlabel("Time [s]")
            plt.ylabel("Error norm [mm]")
            plt.legend()
            plt.grid(True)

            out_png = os.path.join(self.processor.run_paths["plots"], "errors.png")
            plt.savefig(out_png, dpi=150, bbox_inches="tight")
            plt.close()

            # ---- JACOBIAN Δ PLOT ----
            deltas = []
            prev = None
            for jac in self.processor.controller.container.previous_jacobians:
                if prev is not None:
                    deltas.append(np.linalg.norm(jac - prev))
                prev = jac

            plt.figure()
            plt.plot(range(len(deltas)), deltas, marker="o")
            plt.xlabel("Step")
            plt.ylabel("Jacobian Δ (Frobenius norm)")
            plt.title("Change in Jacobian over time")

            out_png = os.path.join(self.processor.run_paths["plots"], "jacobian_deltas.png")
            plt.savefig(out_png, dpi=150, bbox_inches="tight")
            plt.close()

        except Exception as e:
            rp.logerr(f"Failed to generate plots: {e}")
            df = None

        # -------------------- REWIND --------------------
        try:
            # Total steps needed back to home
            ins_steps = - (workspace_mapper.actual_insertion     - workspace_mapper.Min_INS)
            ml_steps  = - (workspace_mapper.actual_ML_actuation - workspace_mapper.starting_ML) 
            ap_steps  = - (workspace_mapper.actual_AP_actuation - workspace_mapper.starting_AP)

            ML_STEP  = 20
            AP_STEP  = 15
            INS_STEP = 150

            rp.loginfo(f"Rewinding robot (ML: {ml_steps} → AP: {ap_steps} → INS: {ins_steps})...")

            # ------------ ML rewind ------------
            if ml_steps != 0:
                ml_sign = 1 if ml_steps > 0 else -1
                total = abs(ml_steps)

                for i in range(total // ML_STEP):
                    self.communicator.send_steps_to_arduino_go_back_ml(
                        ml_sign * ML_STEP
                    )
                    print(f"remaining {total - i * ML_STEP} ML steps")

                rem = total % ML_STEP
                
                if rem:
                    self.communicator.send_steps_to_arduino_go_back_ml(
                        ml_sign * rem
                    )
            else:
                rp.loginfo("ML already at start — no rewind needed.")

            # ------------ AP rewind ------------
            if ap_steps != 0:
                ap_sign = 1 if ap_steps > 0 else -1
                total = abs(ap_steps)

                for i in range(total // AP_STEP):
                    self.communicator.send_steps_to_arduino_go_back_ap(
                        ap_sign * AP_STEP
                    )
                    print(f"remaining {total - i * AP_STEP} AP steps")
                rem = total % AP_STEP

             

                if rem:
                    self.communicator.send_steps_to_arduino_go_back_ap(
                        ap_sign * rem
                    )
            else:
                rp.loginfo("AP already at start — no rewind needed.")

            # ------------ INS rewind ------------
            if ins_steps != 0:
                ins_sign = 1 if ins_steps > 0 else -1
                total = abs(ins_steps)

                for i in range(total // INS_STEP):
                    self.communicator.send_steps_to_arduino_go_back_ins(
                        ins_sign * INS_STEP
                    )
                    print(f"remaining {total - i * INS_STEP} INS steps")

                rem = total % INS_STEP
                

                if rem:
                    self.communicator.send_steps_to_arduino_go_back_ins(
                        ins_sign * rem
                    )
            else:
                rp.loginfo("INS already at start — no rewind needed.")

            rp.loginfo("Robot successfully brought back to start position.")

        except Exception as e:
            rp.logerr(f"Failed to rewind robot: {e}")

        # -------------------- SHUTDOWN --------------------
        try:
            rp.signal_shutdown("Stop requested and finalize completed.")
        except Exception:
            pass

        
        
        
        
        
        
        
        
        

