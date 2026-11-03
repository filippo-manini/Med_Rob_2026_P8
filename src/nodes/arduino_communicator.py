#!/usr/bin/env python3

import rospy as rp
from std_msgs.msg import String, Float64MultiArray, Float64, Float32 
from models.msg import Position, Displacement
import math
import numpy as np
import scipy.optimize
import tf
import os
import json
from scipy import integrate
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from rospy_tutorials.msg import Floats
from ndi_aurora_msgs.msg import AuroraDataVector
import time 
import csv
import pandas as pd
import sympy as sp # For Jacobian matrix
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose, Point32, PointStamped
#from dynamixel_sdk_examples.msg import SetPosition
from collections import deque, namedtuple
#from utils import quaternion_to_rotation_matrix
from std_msgs.msg import Float32, Float64, Int32MultiArray, String, Float64MultiArray,Int32
from sensor_msgs.msg import PointCloud, ChannelFloat32
from visualization_msgs.msg import Marker
from scipy.signal import butter, filtfilt, find_peaks, argrelextrema
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Path
from std_msgs.msg import Header
# import MPC and estimators
import sys
import os


from scipy.spatial.transform import Rotation as R
   
class ArduinoCommunicator():
    def __init__(self):
        self.run_paths = None
        self.aurora_processing = None
        self.publisher = rp.Publisher("displacements_VC", Int32MultiArray, queue_size=10)
        self.reset_publisher = rp.Publisher("reset_motors", Int32, queue_size=10)
        self.subscriber = rp.Subscriber("Finish_Actuation", Int32, self.arduino_actuated)
        self.arduino_done = False
        self.steps_tracker = {"LIN": 0, "ML": 0, "AP": 0}
        self.LIN_MIN=-10000
        self.LIN_MAX=1000
        self.ML_MIN=-1000
        self.ML_MAX=1000
        self.AP_MIN=-1000
        self.AP_MAX=1000
        self.ML_speed = 20
        self.AP_speed = 20
        self.INS_speed = 100
    
    
    def check_CC_parameters(self,delta_steps):
        base_pos = self.aurora_processing.base_pose.copy()
        tip_pos = self.aurora_processing.tip_pose.copy()
        
        l, phi, k, theta = self.aurora_processing.compute_cc_parameters(tip_pos[:3,3], base_pos[:3, 3])
        print(f"Computed length: {l:.2f} mm")
        if l < 20:
            print(f"Computed length: {l:.2f} mm")
            if delta_steps[0] < 0:
                print(" Base pose al momento dell'errore:", base_pos[:3, 3])
                print(" Tip pose al momento dell' errore:", tip_pos[:3, 3])
                print(" \r\n ")
                print(" \r\n ")
                print(" \r\n ")
                print(" \r\n ")
                
                print("Minimum catheter length exceeded: cannot actuate to retract the catheter any further press t to go back")
                
                print(" \r\n ")
                print(" \r\n ")
                print(" \r\n ")
                print(" \r\n ")
 
                    
                    
                    
        if l > 70:
            print(f"Computed length: {l:.2f} mm")
            base_pos = self.aurora_processing.base_pose.copy()
            tip_pos = self.aurora_processing.tip_pose.copy()
            l, phi, k, theta = self.aurora_processing.compute_cc_parameters(tip_pos[:3,3], base_pos[:3, 3])
            if delta_steps[0] > 0 and l > 70:
                print(" Base pose al momento dell'errore:", base_pos[:3, 3])
                print(" Tip pose al momento dell' errore:", tip_pos[:3, 3])
                print(" \r\n ")
                print(" \r\n ")
                print(" \r\n ")
                print(" \r\n ")
                
                print("Maximum catheter length exceeded: cannot actuate to insert the catheter any further press t to go back")
                
                print(" \r\n ")
                print(" \r\n ")
                print(" \r\n ")
                print(" \r\n ")

                    
        return delta_steps    

    def send_steps_to_arduino_calibration(self, delta_steps):
        """Send delta steps to Arduino."""
        
        print("delta steps: ",delta_steps)
        
        
        steps_message = Int32MultiArray(data=delta_steps)
        
        print("steps message: ",steps_message)
        self.publisher.publish(steps_message)
        time_delay = max( (abs(delta_steps[0]) / self.INS_speed, abs(delta_steps[1]) / self.ML_speed, abs(delta_steps[2]) / self.AP_speed)) + 0.1
        rp.sleep(time_delay)
        
        
    def send_steps_to_arduino_go_back(self, delta_steps):
            """Send delta steps to Arduino."""
            
            print("delta steps: ",delta_steps)
            ins_message = delta_steps[0]
            ml_message = delta_steps[1]
            ap_message = delta_steps[2]

            ######################################################################################################################
            #######            To avoid bad hysteresis behavior rewind actuations in the opposite order i.e. first         #######
            #######            AP, then ML and finally insertion                                                           #######
            ######################################################################################################################



            # AP #

            delta_steps = np.zeros(3, dtype=int)
            delta_steps[2] = ap_message
            steps_message = Int32MultiArray(data=delta_steps)
            self.publisher.publish(steps_message)
            self.wait_for_arduino()
            time_delay = abs(delta_steps[2]) / self.AP_speed
            rp.sleep(time_delay)

            # ML #

            delta_steps = np.zeros(3, dtype=int)
            delta_steps[1] = ml_message
            steps_message = Int32MultiArray(data=delta_steps)
            self.publisher.publish(steps_message)
            self.wait_for_arduino()
            time_delay = abs(delta_steps[1]) / self.ML_speed
            rp.sleep(time_delay)

            # INS #

            delta_steps = np.zeros(3, dtype=int)
            delta_steps[0] = ins_message
            steps_message = Int32MultiArray(data=delta_steps)
            self.publisher.publish(steps_message)
            self.wait_for_arduino()
            time_delay = abs(delta_steps[0]) / self.INS_speed
            rp.sleep(time_delay)


    def send_steps_to_arduino_go_back_ins(self, delta_step):
            """Send delta steps to Arduino."""
            



            # INS #

            delta_steps = np.zeros(3, dtype=int)
            delta_steps[0] = delta_step
            steps_message = Int32MultiArray(data=delta_steps)
            self.publisher.publish(steps_message)
            print("publishing :", steps_message)

            self.wait_for_arduino()
            time_delay = abs(delta_steps[0]) / self.INS_speed 
            rp.sleep(time_delay)

    def send_steps_to_arduino_go_back_ml(self, delta_step):
            """Send delta steps to Arduino."""
            



            # INS #

            delta_steps = np.zeros(3, dtype=int)
            delta_steps[1] = delta_step
            steps_message = Int32MultiArray(data=delta_steps)
            print("publishing :", steps_message)
            self.publisher.publish(steps_message)
            self.wait_for_arduino()
            time_delay = abs(delta_steps[1]) / self.ML_speed 
            rp.sleep(time_delay)


    def send_steps_to_arduino_go_back_ap(self, delta_step):
            """Send delta steps to Arduino."""
            
            # INS #

            delta_steps = np.zeros(3, dtype=int)
            delta_steps[2] = delta_step
            steps_message = Int32MultiArray(data=delta_steps)
            print("publishing :", steps_message)

            self.publisher.publish(steps_message)
            self.wait_for_arduino()
            time_delay = abs(delta_steps[2]) / self.AP_speed
            rp.sleep(time_delay)


    def send_steps_to_arduino(self, delta_steps, target, valve_pos):
        """Send delta steps to Arduino and log current state to CSV."""

        # Validate delta steps
        delta_steps = self.check_CC_parameters(delta_steps)
        elapsed_time = time.time() - self.aurora_processing.start_time
        # Prepare data row
        data_row = {
            "time": elapsed_time,
            "timestamp" : time.time(),
            "base_pos_x": self.aurora_processing.base_pose[0, 3],
            "base_pos_y": self.aurora_processing.base_pose[1, 3],
            "base_pos_z": self.aurora_processing.base_pose[2, 3],
            "tip_pos_x": self.aurora_processing.tip_pose[0, 3],
            "tip_pos_y": self.aurora_processing.tip_pose[1, 3],
            "tip_pos_z": self.aurora_processing.tip_pose[2, 3],
            "valve_pos_x": valve_pos[0],
            "valve_pos_y": valve_pos[1],
            "valve_pos_z": valve_pos[2],
            "target_tip_pos_x": target[0],
            "target_tip_pos_y": target[1],
            "target_tip_pos_z": target[2],
            "ACTUATION_INS": delta_steps[0],
            "ACTUATION_ML": delta_steps[1],
            "ACTUATION_AP": delta_steps[2],
            # placeholders, will update after actuation
            "reached_tip_pos_x": 0,
            "reached_tip_pos_y": 0,
            "reached_tip_pos_z": 0,
            "normalized_position_error": 0,
            "position_error": 0,
            "target_error": 0
        }
        norm_of_the_displacement = np.linalg.norm(self.aurora_processing.tip_pose[:3, 3] - target)
        # Send delta steps to Arduino
        steps_message = Int32MultiArray(data=delta_steps)
        self.publisher.publish(steps_message)
        time_delay = max( (abs(delta_steps[0]) / self.INS_speed, abs(delta_steps[1]) / self.ML_speed, abs(delta_steps[2]) / self.AP_speed)) + 0.1
        rp.sleep(time_delay)
        self.wait_for_arduino()
        
        
        
        
        
        # Update row with reached tip and errors
        data_row.update({
            "reached_tip_pos_x": self.aurora_processing.tip_pose[0, 3],
            "reached_tip_pos_y": self.aurora_processing.tip_pose[1, 3],
            "reached_tip_pos_z": self.aurora_processing.tip_pose[2, 3],
            "position_error": np.linalg.norm(self.aurora_processing.tip_pose[:3, 3] - target),
            #"target_error": np.linalg.norm(self.aurora_processing.tip_pose[:3, 3] - valve_pos),
            "target_error": self.aurora_processing.target_error,
            "normalized_position_error": np.linalg.norm((self.aurora_processing.tip_pose[:3, 3] - target) / norm_of_the_displacement),

        })

        # Append row to CSV
        with open(self.run_paths["results_csv"], "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.run_paths["fieldnames_dict"])
            writer.writerow(data_row)

    def send_steps_to_arduino_dead_zone(self, delta_steps):
        """Send delta steps to Arduino and log current state to CSV."""

        # Validate delta steps
        #delta_steps = self.check_CC_parameters(delta_steps)

        # Send delta steps to Arduino
        steps_message = Int32MultiArray(data=delta_steps)
        self.publisher.publish(steps_message)
        time_delay = (abs(delta_steps[0]) / self.INS_speed +  abs(delta_steps[1]) / self.ML_speed + abs(delta_steps[2]) / self.AP_speed) 
        
        rp.sleep(time_delay)
        self.wait_for_arduino()

       
        
        




    def wait_for_arduino(self):
        """Wait for Arduino to signal completion."""
        rate = rp.Rate(100)  # Check for completion every 10 milliseconds
        while not self.arduino_done and not rp.is_shutdown():
            #rp.loginfo("Waiting for Arduino to complete actuation...")
            rate.sleep()

    def arduino_actuated(self, msg):
        """Callback when Arduino signals completion."""
        self.arduino_done = True
        #rp.loginfo(f"Arduino actuation completed: {msg.data}")
        
    def send_reset_command(self):
        """Send a reset command to Arduino."""
        print("Sending reset command to Arduino")
        reset_message = Int32()
        reset_message.data = 1  # Command to reset all motors
        self.reset_publisher.publish(reset_message)
        self.wait_for_arduino()
