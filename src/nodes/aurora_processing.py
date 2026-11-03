
#!/usr/bin/env python3

import rospy as rp
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from ndi_aurora_msgs.msg import AuroraDataVector
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R
import numpy as np
import tf
import os
import matplotlib.pyplot as plt
import time
from collections import deque, namedtuple
import yaml
import os
import csv



AuroraData = namedtuple('AuroraData', 'pos orientation')



class AuroraProcessing():     
    rp.init_node('aurora_processing_node', anonymous=True)
    AURORA_CHANNEL1 = 0  #Channel 1 is the TIP
    AURORA_CHANNEL2 = 1  #Channel 2 is the BASE
    AURORA_CHANNEL3 = 2
    MAX_HISTORY = 10
    AURORA_THR_POS = 20
    AURORA_THR_ORI = 1.4
    NOISE_THRESHOLD = 7
    MIN_AP=1
    MAX_AP= 2200 #set the maxx!!!!!!!!!!!!!!!!
    

    def __init__(self, calibrated = True):
        print("Initializing AuroraProcessing...")
        self.data = None
        self.tip_pose = np.eye(4)
        self.base_pose = np.eye(4)
        self.valve_pose = np.eye(4)
        self.target_error = 1000 # initialize not just to have 0
        self.min_target_error = 1000
        self.T_tip_calibration = np.eye(4)
        self.T_base_calibration = np.eye(4)
        self.tip_history = deque(maxlen=self.MAX_HISTORY)
        self.base_history = deque(maxlen=self.MAX_HISTORY)
        self.aurora_filter_count = 0
        self.steps = Int32MultiArray()
        self.flag_calibration_done = calibrated
        self.calibration_data = {"tip": [], "base": []}  # Dati della calibrazione
        self.start_time = rp.get_time()
        self.run_paths = None
        self.control_loop_started = False 
        
        # Initialize subscriber to receive Aurora data
        self.subscriber = rp.Subscriber("aurora_data", AuroraDataVector, self.callback)
        # Initialize publishers to send pose data
        self.tip_pub = rp.Publisher("tip_pose_topic", PoseStamped, queue_size=10)
        self.base_pub = rp.Publisher("base_pose_topic", PoseStamped, queue_size=10)
        # Initialize marker publisher
        self.marker_pub = rp.Publisher("catheter_shape_marker", Marker, queue_size=10)

        # Initialize publisher to Arduino
        self.arduino_communicator =  None
        self.controller = None

        
        self.start_time = time.time()  # save reference start
          
        
        # Attributes set by the main node
        self.args = None
        self.kinematics = None
        self.hysteresis_compensation_ml = None
        self.hysteresis_compensation_ap = None
        self.motor_steps = None
        #self.tip_pub = None
        #self.base_pub = None
        self.pub_set_position = None
        
    def normalize_quat(self, q_xyzw):
        """Normalize quaternion(s) and enforce qw >= 0 for sign consistency."""
        q = np.array(q_xyzw, dtype=float)
        q /= np.linalg.norm(q)
        if q[3] < 0:
            q = -q
        return q

    def stack_T(self, q, p, orthonormalize=True, tol=1e-6):
        """Quaternion + position → 4x4 transform.
        
        Builds a 4×4 homogeneous transformation matrix from a quaternion `q`
        and translation vector `p`. Optionally enforces orthonormality of the
        rotation matrix if small numerical errors are present.
        """
        # Convert quaternion to rotation matrix
        Rm = R.from_quat(q).as_matrix()

        if orthonormalize:
            # Check orthonormality
            err = np.linalg.norm(Rm.T @ Rm - np.eye(3))
            det = np.linalg.det(Rm)
            if abs(err) > tol or not np.isclose(det, 1.0, atol=tol):
                # Fix via SVD only if necessary
                U, _, Vt = np.linalg.svd(Rm)
                if np.linalg.det(U @ Vt) < 0:
                    U[:, -1] *= -1
                Rm = U @ Vt

        # Build 4×4 transform
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = p
        return T
    
    def callback(self, aurora_msg):

            tip_data = self._extract_data(aurora_msg.data[self.AURORA_CHANNEL1])  
            base_data = self._extract_data(aurora_msg.data[self.AURORA_CHANNEL2])       
            print(tip_data)   
            self.aurora_filter_count = 0
            self._update_history(tip_data, base_data)
            self._update_poses()
            if True:
                self.publish_pose(self.tip_pub, self.tip_pose, 'world')
                self.publish_pose(self.base_pub, self.base_pose, 'world')                
                
    def _update_history(self, tip_data, base_data):
        self.tip_history.append(tip_data)
        self.base_history.append(base_data)
        
    def _update_poses(self):
        # --- Tip ---
        tip_pos = np.mean([d.pos for d in self.tip_history], axis=0)
        #tip_quats = [normalize_quat(d.orientation) for d in self.tip_history]

        #tip_quat = np.mean([quat for quat in tip_quats], axis=0) 
        tip_quat = np.mean([d.orientation for d in self.tip_history], axis=0) 

        T_tip_meas = self.stack_T(tip_quat, tip_pos) @ self.T_tip_calibration

        # --- Base ---
        base_pos = np.mean([d.pos for d in self.base_history], axis=0)
        #base_quats = [normalize_quat(d.orientation) for d in self.base_history]
        #base_quat = np.mean([quat for quat in base_quats], axis=0)
        base_quat = np.mean([d.orientation for d in self.base_history], axis=0)

        T_base_meas = self.stack_T(base_quat, base_pos) @ self.T_base_calibration

        # --- Relative tip in base ---
        self.tip_pose = np.linalg.inv(T_base_meas) @ T_tip_meas

        pos = self.tip_pose[:3, 3]
                    

        # --- Base pose identity (to keep same API as before) ---
        self.base_pose = np.linalg.inv(T_base_meas) @ T_base_meas

    def _extract_data(self, channel_data):
                return AuroraData(
                    pos=np.array([channel_data.position.x, channel_data.position.y, channel_data.position.z]),
                    orientation=np.array([channel_data.orientation.x, channel_data.orientation.y,
                                        channel_data.orientation.z, channel_data.orientation.w])
                )
    def publish_pose(self, publisher, pose, frame_id): #questo per pubblicare in rviz
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp = rp.Time.now()
        
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = pose[:3, 3] / 10
        quaternion = tf.transformations.quaternion_from_matrix(pose)
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, \
        pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = quaternion / np.linalg.norm(quaternion)

        publisher.publish(pose_msg)

if __name__ == '__main__':

    rp.init_node('aurora_processing_node', anonymous=True)

    aurora = AuroraProcessing()

    rp.spin()