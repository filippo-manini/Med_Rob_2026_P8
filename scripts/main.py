#!/usr/bin/env python3


import rospy as rp
import os
import sys


sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'Pos1_J_UPD.yaml')
from ConfigParser import Parser
from utilities import *
parser = Parser(config_path)
parser.parse()

# Now retrieve components using the ConfigParser class
processor = parser.get_component("processor")
communicator = parser.get_component("communicator")
container = parser.get_component("container")
jacobian_estimator = parser.get_component("jacobian_estimator")
trajectory_estimator = parser.get_component("trajectory_estimator")
valve_predictor = parser.get_component("valve_predictor")
pid_controller = parser.get_component("pid_controller")
controller = parser.get_component("controller")

rp.init_node('aurora_processing_node', anonymous=True)

# ======= INTERDEPENDENCIES ========
processor.controller = controller
controller.processor = processor
processor.arduino_communicator = communicator
controller.communicator = communicator
communicator.aurora_processing = processor

# ======= DEBUGGING INIT ========
print(f"processor: {processor}")
print(f"controller: {controller}")

if not processor.flag_calibration_done:
    processor.calibrate_aurora()

rp.sleep(2)  # short delay before control

print(f"Before ComputeClosedLoopControl - controller.aurora_processing: {controller.processor}")
print(f"Before ComputeClosedLoopControl - processor.tip_pose: {processor.tip_pose}")

user_input = ""
while user_input.strip().lower() != "start":
    user_input = input("Type 'start' to create directories and begin the simulation: ")
    if user_input.strip().lower() != "start":
        print("Invalid input. Please type 'start' to proceed.")



# Setup workspace
paths = create_run_workspace(config=config_path)
init_logging(paths["log_file"])
logging.info("Workspace created at: %s", paths["run_dir"])

# Pass paths to components
communicator.run_paths = paths
processor.run_paths = paths

plotter = CSVPlotter()
plotter.processor = processor
plotter.communicator = communicator
controller.plotter = plotter

rp.loginfo("Node running. Press Ctrl+C to stop and generate plots.")

controller.Compute_MPC_Control()

rp.spin()
