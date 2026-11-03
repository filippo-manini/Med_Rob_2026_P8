

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
        self.drop_value = 0
        self.model_identified = False
        self.wait_for_sync = 0
        self.valve_meas = []
        self.valve_pred = []
        self.generate_static = False
        self.start_time = rp.get_time()
        self.generate_dynamic = False
        self.valve_base = None
        self.valve_dir = None 
        self.valve_amplitude = None 
        self.valve_frequency = None
        self.run_paths = None
        self.control_loop_started = False 
        
        # Initialize subscriber to receive Aurora data
        #self.subscriber = rp.Subscriber("aurora_data", AuroraDataVector, self.callback)
        # Initialize publishers to send pose data
        self.tip_pub = rp.Publisher("tip_pose_topic", PoseStamped, queue_size=10)
        self.base_pub = rp.Publisher("base_pose_topic", PoseStamped, queue_size=10)
        # Initialize marker publisher
        self.marker_pub = rp.Publisher("catheter_shape_marker", Marker, queue_size=10)

        # Initialize publisher to Arduino
        self.arduino_communicator =  None
        self.controller = None
        # Call calibration at initialization
        if not self.flag_calibration_done:
           print("calibrate aurora !!!")
        else: 
            print("already calibrated")
            self.load_calibration_data()
        print("AuroraProcessing initialized.")
        
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
            self.aurora_filter_count = 0
            self._update_history(tip_data, base_data)
            self._update_poses_EMA()
            if True:
                self.publish_pose(self.tip_pub, self.tip_pose, 'world')
                self.publish_pose(self.base_pub, self.base_pose, 'world')                
                
                # Calculate and publish catheter shape
                self.publish_catheter_shape()

                

    def compute_cc_parameters(self, tip_pos, base_pos):
        l = np.linalg.norm(tip_pos - base_pos)
        phi = np.arctan2(tip_pos[1], tip_pos[0])
        k = 2 * np.sqrt(tip_pos[0]**2 + tip_pos[1]**2) / (tip_pos[0]**2 + tip_pos[1]**2 + tip_pos[2]**2)
        theta = np.arccos(1 - k * np.sqrt(tip_pos[0]**2 + tip_pos[1]**2))
        return l, phi, k, theta

    def generate_cc_curve(self, tip_pos, base_pos, num_points=100):
        l, phi, k, theta = self.compute_cc_parameters(tip_pos, base_pos)

        points = []
        for s in np.linspace(0, l, num_points):
            x = (s / l)* 1/k * np.cos(phi) * (1 - np.cos(theta * (s / l)))
            y = (s / l)* 1/k * np.sin(phi) * (1 - np.cos(theta * (s / l)))
            z = (s / l)* 1/k * np.sin(theta * (s / l))
            points.append([x, y, z])
        return np.array(points)

    def publish_cc_curve(self, points):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rp.Time.now()
        marker.ns = "catheter_shape"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # Use LINE_STRIP to connect the points
        marker.action = Marker.ADD
        marker.scale.x = 0.3  # Line width
        marker.scale.y = 0.3  # Line width
        marker.scale.z = 0.3  # Line width
        marker.color.a = 10  # Alpha
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5

        for point in points:
            p = Point()
            p.x = point[0]/10
            p.y = point[1]/10
            p.z = point[2]/10
            marker.points.append(p)

        self.marker_pub.publish(marker)

    def publish_catheter_shape(self):
        tip_position = self.tip_pose[:3, 3]
        base_position = self.base_pose[:3, 3]
        points = self.generate_cc_curve(tip_position, base_position)
        self.publish_cc_curve(points)
    
    def _extract_data(self, channel_data):
            return AuroraData(
                pos=np.array([channel_data.position.x, channel_data.position.y, channel_data.position.z]),
                orientation=np.array([channel_data.orientation.x, channel_data.orientation.y,
                                    channel_data.orientation.z, channel_data.orientation.w])
            )

    def _extract_data_reference(self, channel_data):
 

        # Posizione trasformata (corretta)
        pos = np.array([
            -channel_data.position.z,  # X nuovo = -Z aurora
            channel_data.position.y,   # Y nuovo = Y aurora
            channel_data.position.x    # Z nuovo = X aurora
        ])

        # Quaternion originale (Aurora)
        q_aurora = np.array([
            channel_data.orientation.x,
            channel_data.orientation.y,
            channel_data.orientation.z,
            channel_data.orientation.w
        ])

        # Quaternione di rotazione Aurora -> Nuovo (90° attorno a Y)
        q_R = tf.transformations.quaternion_from_euler(0, np.deg2rad(90), 0)

        # ✅ Corretto: q_N = q_aurora ⊗ q_R
        q_N = tf.transformations.quaternion_multiply(q_aurora, q_R)

        return AuroraData(pos=pos, orientation=q_N)


    def _is_noise(self, tip_data, base_data, valve_data):
            if not self.tip_history or self.aurora_filter_count >= self.NOISE_THRESHOLD:
                return False
            prev_tip, prev_base, prev_valve = self.tip_history[-1], self.base_history[-1], self.valve_history[-1]
            return any(np.abs(tip_data.pos - prev_tip.pos) > self.AURORA_THR_POS) or \
                any(np.abs(tip_data.orientation - prev_tip.orientation) > self.AURORA_THR_ORI) or \
                any(np.abs(base_data.pos - prev_base.pos) > self.AURORA_THR_POS) or \
                any(np.abs(base_data.orientation - prev_base.orientation) > self.AURORA_THR_ORI) or \

    def _update_history(self, tip_data, base_data, valve_data):
        self.tip_history.append(tip_data)
        self.base_history.append(base_data)
        if self.drop_value == 0:
           self.valve_history.append(valve_data)
        self.drop_value += 1
        self.drop_value = self.drop_value % 4
        #self.drop_value = 0 

    
    
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

        # --- Static generation ---


    def _update_poses_EMA(self, alpha_tip=0.10):
            # --- Tip ---
            tip_pos = np.mean([d.pos for d in self.tip_history], axis=0)
            tip_quat = np.mean([d.orientation for d in self.tip_history], axis=0)
            
            #use only last
            
            tip_pos = self.tip_history[-1].pos
            tip_quat = self.tip_history[-1].orientation
            
            
            T_tip_meas = self.stack_T(tip_quat, tip_pos) @ self.T_tip_calibration

            # --- Base ---
            base_pos = np.mean([d.pos for d in self.base_history], axis=0)
            base_quat = np.mean([d.orientation for d in self.base_history], axis=0)
            T_base_meas = self.stack_T(base_quat, base_pos) @ self.T_base_calibration

            # --- Relative tip in base ---
            new_tip_pose = np.linalg.inv(T_base_meas) @ T_tip_meas
            new_tip_pos = new_tip_pose[:3, 3]

            # --- EMA filtering sul tip ---
            filtered_tip_pos = alpha_tip * new_tip_pos + (1 - alpha_tip) * getattr(self, 'last_filtered_tip', new_tip_pos)
            self.last_filtered_tip = filtered_tip_pos

            # Aggiorno tip_pose con la posizione filtrata
            self.tip_pose = new_tip_pose.copy()
            self.tip_pose[:3, 3] = filtered_tip_pos

            

            # --- Base pose identity (compatibilità API) ---
            self.base_pose = np.linalg.inv(T_base_meas) @ T_base_meas
            # --- Static generation ---
            if self.generate_static:
                
                self.valve_pose[:3, 3] = self.valve_base
                print("valve pose!!", self.valve_pose)
                self.generate_static = False
                    
            if self.generate_dynamic:
                point = self.valve_base

                # parameters
                
                delta_t = rp.get_time()  - self.start_time    # time [s]

            # call
                self.valve_pose[:3, 3] = self.move_valve(self.valve_amplitude, self.valve_frequency, point, delta_t, self.valve_dir)
                
            # --- Aggiornamento target error ---
            self.target_error = np.linalg.norm(self.tip_pose[:3, 3] - self.valve_pose[:3, 3])
                
            if self.target_error < self.min_target_error:
                self.min_target_error = self.target_error
                #print(f"!!!! MIN TARGET ERROR DETECTED !!! {self.min_target_error}")
                if self.min_target_error < self.controller.stopping_threshold:
                    self.controller.plotter.stop_requested = True
                
            valve_pos = self.valve_pose[:3, 3]   
            valve_quat = tf.transformations.quaternion_from_matrix(self.valve_pose) 
            data_row = {
                "time": time.time() - self.start_time,
                "timestamp": time.time(),
                "base_pos_x": self.base_pose[0, 3],
                "base_pos_y": self.base_pose[1, 3],
                "base_pos_z": self.base_pose[2, 3],
                "tip_pos_x": self.tip_pose[0, 3],
                "tip_pos_y": self.tip_pose[1, 3],
                "tip_pos_z": self.tip_pose[2, 3],
                "tip_pos_qx": valve_quat[0], 
                "tip_pos_qy": valve_quat[1], 
                "tip_pos_qz": valve_quat[2],
                "tip_pos_qw": valve_quat[3],
                "valve_pos_x": valve_pos[0],
                "valve_pos_y": valve_pos[1],
                "valve_pos_z": valve_pos[2],
                "target_error": self.target_error,
                "min_target_error": self.min_target_error
            }
            if self.control_loop_started:                         
                with open(self.run_paths["csv_logs"], "a", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=self.run_paths["fieldnames_logs_dict"])
                    writer.writerow(data_row)
                if rp.get_time() - self.start_time > 600:
                    self.controller.plotter.stop_requested = True
                    
      
       
    def publish_pose(self, publisher, pose, frame_id): #questo per pubblicare in rviz
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.header.stamp = rp.Time.now()
        
        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = pose[:3, 3] / 10
        quaternion = tf.transformations.quaternion_from_matrix(pose)
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, \
        pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = quaternion / np.linalg.norm(quaternion)

        publisher.publish(pose_msg)
        
    def publish_point_valve(self, publisher, point, frame_id):
        point_msg = PointStamped()
        point_msg.header.frame_id = frame_id
        point_msg.header.stamp = rp.Time.now()
        
        point_msg.point.x = point[0] / 10
        point_msg.point.y = point[1] / 10
        point_msg.point.z = point[2] / 10

        publisher.publish(point_msg)
        
    def calibrate_aurora(self):
        
        print('Calibrating Aurora')
        # Collect data from Aurora while moving the AP metti dentro gli step che vuoi mandare ad arduino
        #talk to arduino                
        positions = np.linspace(self.MIN_AP, self.MAX_AP, 50).astype(int)
        
        aurora_tip_data, aurora_base_data = self.collect_aurora_data(positions) #collect data from aurora

        # Process the collected data
        normal = - self.process_aurora_data(aurora_tip_data, aurora_base_data)
        self.calculate_calibration_transformations(normal)

        # Reset and save calibration
        #aggiungi linea per tornare all'origine
        self.save_calibration_data()
        self.set_processor_calibrated(True)
        
        ## aggiungi per tornare indietro? #######################################
        self.arduino_communicator.send_reset_command()
        self.flag_calibration_done = True
        print('Calibration finished, motors back to the initial position ready to control')

    def collect_aurora_data(self, positions): 
        aurora_tip_data = np.zeros((len(positions), 3))
        aurora_base_data = np.zeros((len(positions), 3))
        delta_step = [0, 0, 0]
        i=0
        j = 0
        for i in range(len(positions)):
            delta_step=[0,0,positions[1]]
            self.arduino_communicator.send_steps_to_arduino_calibration(delta_step) #pubblica step
            #rp.sleep(1)
            aurora_tip_data[i] = self.tip_pose[0:3, 3] #save aurora dati
            aurora_base_data[i] = self.base_pose[0:3, 3]
            

        return aurora_tip_data, aurora_base_data
    
    
    
    def collect_data_for_jacobian_ml(self, delta_ml, number_of_valid_samples):
        valid_samples = 0  # counter for valid (non-dead-zone) samples
        starting_position = self.tip_pose[0:3, 3].copy()

        for i in range(10):
            rp.sleep(0.1)
        
            self.controller.container.detect_dead_zone_EMA(
                    np.array(([0, -20, 0])), starting_position, self.tip_pose[0:3, 3]
                )
            starting_position = self.tip_pose[0:3, 3]
        while valid_samples < number_of_valid_samples:
            

            actuation = [0, delta_ml, 0]
                
                
            self.arduino_communicator.send_steps_to_arduino_calibration(actuation)
            # rp.sleep(0.1)  # optional: wait for stability

            dead_zone_not_found = self.controller.container.detect_dead_zone_EMA(
                actuation, starting_position, self.tip_pose[0:3, 3]
            )

            if dead_zone_not_found:  # == 1 means movement occurred
                self.controller.container.update_observations(
                    actuation, starting_position, self.tip_pose[0:3, 3], 0
                )
                valid_samples += 1  # only count valid (non-dead-zone) data
                
            starting_position = self.tip_pose[0:3, 3].copy()  # current tip position

            # Optional: log progress
            print(f"Collected {valid_samples}/{number_of_valid_samples} valid samples.")

            
            

        
        
    def collect_data_for_jacobian_ap(self, delta_ap, number_of_valid_samples):
        valid_samples = 0  # counter for valid (non-dead-zone) samples
        starting_position = self.tip_pose[0:3, 3].copy()
        for i in range(10):
                rp.sleep(0.1)
            
                self.controller.container.detect_dead_zone_EMA(
                        np.array(([0, -20, 0])), starting_position, self.tip_pose[0:3, 3]
                    )
                starting_position = self.tip_pose[0:3, 3]
        while valid_samples < number_of_valid_samples:
            starting_position = self.tip_pose[0:3, 3].copy()  # current tip position

            actuation = [0, 0, delta_ap]
            self.arduino_communicator.send_steps_to_arduino_calibration(actuation)
            # rp.sleep(0.1)  # optional: wait for stability

            dead_zone_not_found = self.controller.container.detect_dead_zone_EMA(
                actuation, starting_position, self.tip_pose[0:3, 3]
            )

            if dead_zone_not_found:  # == 1 means movement occurred
                self.controller.container.update_observations(
                    actuation, starting_position, self.tip_pose[0:3, 3], 0
                )
                valid_samples += 1  # only count valid (non-dead-zone) data

            # Optional: log progress
            print(f"Collected {valid_samples}/{number_of_valid_samples} valid samples for AP.")

            

        
        
    
    
    def collect_data_for_jacobian_ins(self, delta_ins, number_of_valid_samples):
        valid_samples = 0   # counts only valid (non-dead-zone) samples
        total_steps = 0     # counts all actuation attempts

        print("Collecting data for Jacobian (insertion)...")
        starting_position = self.tip_pose[0:3, 3].copy()
        for i in range(10):
                rp.sleep(0.1)
            
                self.controller.container.detect_dead_zone_EMA(
                        np.array(([0, -20, 0])), starting_position, self.tip_pose[0:3, 3]
                    )
                starting_position = self.tip_pose[0:3, 3]
        while valid_samples < number_of_valid_samples:
            starting_position = self.tip_pose[0:3, 3].copy()
            actuation = [delta_ins, 0, 0]
            self.arduino_communicator.send_steps_to_arduino_calibration(actuation)
            total_steps += 1  # count every actuation
            # rp.sleep(0.1)  # optional stabilization

            dead_zone_not_found = self.controller.container.detect_dead_zone_EMA(
                actuation, starting_position, self.tip_pose[0:3, 3]
            )

            if dead_zone_not_found:  # movement occurred
                self.controller.container.update_observations(
                    actuation, starting_position, self.tip_pose[0:3, 3], 0
                )
                valid_samples += 1
                print(f"Collected {valid_samples}/{number_of_valid_samples} valid samples.")
            starting_position = self.tip_pose[0:3, 3].copy()  # current tip position




    def process_aurora_data(self, tip_data, base_data):
        points = np.concatenate((tip_data, base_data), axis=0).T
        svd = np.linalg.svd(points - np.mean(points, axis=1, keepdims=True))
        print("found with SVD: ",svd)
        left = svd[0]
        normal = left[:, -1]
        print(f'Normal vector: {normal}')
        
        ############################################################################################
        ########################### IF THE CALIBRATION HAS OPPOSITE AXIS ###########################
        ########################### INVERT YOU HAVE TO CHANGE TO -NORMAL ###########################
        ############################################################################################
        
        
        return -normal 
    
    def calculate_calibration_transformations(self, normal):
        T_tip_offset = np.eye(4)
        T_tip_offset[0, 3] = 1.5 
        T_base_offset = np.eye(4)
        T_base_offset[0, 3] = 2.5
        T_tip, T_base = self.compute_transformations(normal, self.tip_pose, self.base_pose)
        self.T_tip_calibration = np.linalg.inv(self.tip_pose) @ T_tip @ np.linalg.inv(T_tip_offset)
        self.T_base_calibration = np.linalg.inv(self.base_pose) @ T_base @ np.linalg.inv(T_base_offset)
        print(f'Tip calibration transformation:\n{self.T_tip_calibration}')
        print(f'Base calibration transformation:\n{self.T_base_calibration}')

    def compute_transformations(self, normal, tip_pose, base_pose):
        T_tip = self.build_transformation_matrix_WIM(normal, tip_pose)
        T_base = self.build_transformation_matrix_WIM(normal, base_pose)
        return T_tip, T_base

    def build_transformation_matrix(self, normal, pose):
        quaternion = pose[0:3, 0] 
        cross_product = np.cross(normal, quaternion)
        T = np.eye(4)
        T[0:3, 0] = cross_product / np.linalg.norm(cross_product)
        T[0:3, 1] = normal / np.linalg.norm(normal)
        T[0:3, 2] = -quaternion / np.linalg.norm(quaternion)
        T[0:3, 3] = pose[0:3, 3]
        return T
    
    
    
    
    def build_transformation_matrix_WIM(self, normal, T):
        x = -T[:3, 0]  # the z axis of the transformation frame is the x axis of the aurora sensor
        y = np.cross(normal, x) # the y axis is the cross product between the normal calculated with SVD and the z axys 
        _T = np.eye(4)
        _T[:3, 0] = y / np.linalg.norm(y) # antero posterior
        _T[:3, 1] = normal / np.linalg.norm(normal) # medio-lateral actuation
        _T[:3, 2] = x / np.linalg.norm(x) # insertion
        _T[:3, 3] = T[:3, 3]
        return _T

    

    
    



    def set_processor_calibrated(self, value: bool):
        # Path to config.yaml
        config_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'config')
        yaml_path = os.path.join(config_dir, 'config.yaml')

        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f'{yaml_path} does not exist')

        # Load existing YAML
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        # Defensive checks
        if 'processor' not in config:
            raise KeyError('processor section missing in config.yaml')

        config['processor'].setdefault('params', {})

        # Update only the calibrated flag
        config['processor']['params']['calibrated'] = value

        # Save back
        with open(yaml_path, 'w') as f:
            yaml.safe_dump(config, f, sort_keys=False)

        print(f"'processor.params.calibrated' set to {value} in config.yaml")


    
    def save_calibration_data(self):
        path = os.path.join(os.path.dirname(__file__), '..', '..', 'config')
        print('file di calibrazione salvato in: ', path)
        if not os.path.exists(path):
            os.makedirs(path)
            
        with open(os.path.join(path, 'aurora_calibration_transformation.txt'), 'w') as f:
            f.write(f'{self.T_tip_calibration}\n{self.T_base_calibration}\n')
             
    def load_calibration_data(self):
        print("file path", os.path.dirname(os.path.abspath(__file__)))
        file_path = os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'aurora_calibration_transformation.txt')
        print(f"Tentativo di apertura file: {os.path.dirname(os.path.abspath(file_path))}")
        if (os.path.exists(file_path)):
             with open(file_path, 'r') as f:
                lines = f.readlines()

                # Process T_tip_calibration
                T_tip_calibration_lines = []
                for line in lines[:4]:
                    # Rimuovi spazi bianchi e caratteri di nuova riga
                    line = line.strip()
                    # Rimuovi i caratteri '[' e ']' dalla riga
                    line = line.replace('[', '').replace(']', '')
                    # Dividi la riga in elementi separati e convertili in float
                    elements = [float(element) for element in line.split()]
                    # Aggiungi la lista di elementi alla lista delle righe di T_tip_calibration
                    T_tip_calibration_lines.append(elements)
                # Converti la lista di liste in un array numpy
                T_tip_calibration = np.array(T_tip_calibration_lines)

                # Process T_base_calibration
                T_base_calibration_lines = []
                for line in lines[4:8]:
                    # Rimuovi spazi bianchi e caratteri di nuova riga
                    line = line.strip()
                    # Rimuovi i caratteri '[' e ']' dalla riga
                    line = line.replace('[', '').replace(']', '')
                    # Dividi la riga in elementi separati e convertili in float
                    elements = [float(element) for element in line.split()]
                    # Aggiungi la lista di elementi alla lista delle righe di T_base_calibration
                    T_base_calibration_lines.append(elements)
                # Converti la lista di liste in un array numpy
                T_base_calibration = np.array(T_base_calibration_lines)

                # Assegna le matrici di calibrazione
                self.T_tip_calibration = T_tip_calibration
                self.T_base_calibration = T_base_calibration
                #self.T_tip_calibration_matrix = np.eye(4)
                #self.T_base_calibration = np.eye(4)

        else:
            print("File di calibrazione non trovato in: ")
            print(file_path)
            
