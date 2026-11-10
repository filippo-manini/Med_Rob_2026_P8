
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
    AURORA_CHANNEL_PEN = 1  # Channel 1: EM-pen (Pointer used for base calibration)    
    MAX_HISTORY = 10
    AURORA_THR_POS = 20
    AURORA_THR_ORI = 1.4
    NOISE_THRESHOLD = 7
    MIN_AP=1
    MAX_AP= 2200 #set the maxx!!!!!!!!!!!!!!!!
    

    def __init__(self, calibrated = False):
        print("Initializing AuroraProcessing...")
        self.data = None
        self.tip_pose = np.eye(4)
        self.base_pose = np.eye(4)
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
        self.latest_aurora_msg = None

        ### NEW 
        # Matrices for transforming between NDI Aurora frame and base frame
        self.T_ndi_base = np.eye(4)
        self.T_base_ndi_inv = np.eye(4)

        # Initialize subscriber to receive Aurora data
        self.subscriber = rp.Subscriber("aurora_data", AuroraDataVector, self.callback)

        # Initialize publishers to send pose data
        self.tip_pub = rp.Publisher("tip_pose_topic", PoseStamped, queue_size=10)
        self.base_pub = rp.Publisher("base_pose_topic", PoseStamped, queue_size=10)
        # Initialize marker publisher
        self.marker_pub = rp.Publisher("catheter_shape_marker", Marker, queue_size=10)

        # Run interactive 2-point base calibration if not calibrated
        if not self.flag_calibration_done:
            self.run_base_calibration_workflow()

        
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
        # Save latest message for the calibration sampler
        self.latest_aurora_msg = aurora_msg

        if not self.flag_calibration_done:
            return

        tip_data = self._extract_data(aurora_msg.data[self.AURORA_CHANNEL1])  
        self.aurora_filter_count = 0
        self.tip_history.append(tip_data)
        self._update_poses()
        
        self.publish_pose(self.tip_pub, self.tip_pose, 'base_frame')
        self.publish_pose(self.base_pub, np.eye(4), 'base_frame')                
                
    def _update_history(self, tip_data, base_data):
        self.tip_history.append(tip_data)
        self.base_history.append(base_data)
        
    def _update_poses(self):
        # Average tip data to reduce magnetic noise
        tip_pos = np.mean([d.pos for d in self.tip_history], axis=0)
        tip_quat = self.normalize_quat(np.mean([d.orientation for d in self.tip_history], axis=0))

        # Tip pose in global NDI frame
        T_tip_meas = self.stack_T(tip_quat, tip_pos) @ self.T_tip_calibration

        # Relative Tip pose in Base frame using precomputed static inverse
        self.tip_pose = self.T_base_ndi_inv @ T_tip_meas

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


    ### NEW FUNCTION
    def compute_base_transform(self, p1, q1, p2):
        # I want to compute the transformation matrix from the NDI Aurora frame to a base frame defined by 
        # two points (p1, p2) and the orientation of the pen at p1 (q1).
    
        # Parameters:
        # p1 (np.array 3x1): Position of Point 1 (Origin of the base / catheter exit)
        # q1 (np.array 4x1): Quaternion of the pen at Point 1 [x, y, z, w]
        # p2 (np.array 3x1): Position of Point 2 (Point located further back on the catheter axis)

        # Returns:
        # T_ndi_base (np.array 4x4): Transformation matrix from NDI
        # T_base_ndi_inv (np.array 4x4): Inverse transformation matrix from base to NDI to calculate the tip position in the base frame

        # 1. Z axis (longitudinal / tangencial): Vector from point 2 to point 1
        vec_z = p1 - p2
        norm_z = np.linalg.norm(vec_z)
        if norm_z < 1e-4:
            raise ValueError("P1 and P2 are too close; cannot define a base frame.")
        z_base = vec_z / norm_z

        # 2. Orientation vector of the pen at P1 (from quaternion)
        # Ensure normalized quaternion
        q1_norm = self.normalize_quat(q1)
        R_pen1 = R.from_quat(q1_norm).as_matrix()
        v_pen = R_pen1[:, 0]

        # 3. X axis: Projection of v_pen onto the plane perpendicular to z_base
        x_proj = v_pen - np.dot(v_pen, z_base) * z_base
        norm_x = np.linalg.norm(x_proj)
        
        # Case limit if the pen is totally aligned with Z
        if norm_x < 1e-4:
            v_alt = np.array([1.0, 0.0, 0.0]) if abs(z_base[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
            x_proj = v_alt - np.dot(v_alt, z_base) * z_base
            norm_x = np.linalg.norm(x_proj)
            
        x_base = x_proj / norm_x

        # 4. Y axis: Right-hand rule (y = z times x)
        y_base = np.cross(z_base, x_base)
        y_base /= np.linalg.norm(y_base)

        # 5. Assembly of the Rotation Matrix (3x3) and Transformation (4x4)
        R_base = np.column_stack((x_base, y_base, z_base))
        
        T_ndi_base = np.eye(4)
        T_ndi_base[:3, :3] = R_base
        T_ndi_base[:3, 3] = p1

        # 6. The inverse of a homogeneous transformation matrix
        T_base_ndi_inv = np.eye(4)
        T_base_ndi_inv[:3, :3] = R_base.T
        T_base_ndi_inv[:3, 3] = -R_base.T @ p1

        return T_ndi_base, T_base_ndi_inv


    ### NEW FUNCTION
    def sample_pen_point(self, point_name="P1", num_samples=100):
        # 1. Muestra un mensaje y pausa el programa hasta que pases ENTER
        input(f"\nColoca el lápiz en el {point_name} y pulsa ENTER...")
        print(f"Tomando {num_samples} lecturas del sensor...")

        samples_pos = []
        samples_quat = []
        rate = rp.Rate(100)  

        # 2. Bucle para guardar datos hasta llegar a las 100 muestras
        while len(samples_pos) < num_samples and not rp.is_shutdown():
            if self.latest_aurora_msg is not None:
                # Extrae la posición y el cuaternión del lápiz (Canal 1)
                pen_data = self._extract_data(self.latest_aurora_msg.data[self.AURORA_CHANNEL_PEN])
                
                # Guarda la lectura si no está vacía [0, 0, 0]
                if np.linalg.norm(pen_data.pos) > 0:
                    samples_pos.append(pen_data.pos)
                    samples_quat.append(pen_data.orientation)
            rate.sleep()

        # 3. Calcula la media de las posiciones y cuaterniones guardados
        mean_pos = np.mean(samples_pos, axis=0)
        mean_quat = self.normalize_quat(np.mean(samples_quat, axis=0))

        print(f"Punto {point_name} guardado correctamente.")
        return mean_pos, mean_quat

    ### NEW FUNCTION
    def run_base_calibration_workflow(self):
        print("=== BEGINNING OF BASE CALIBRATION ===")

        # 1. Espera a que lleguen los primeros datos de Aurora
        while self.latest_aurora_msg is None and not rp.is_shutdown():
            rp.sleep(0.1)

        # 2. Captura el Punto 1 (boquilla de salida: saca posición p1 y cuaternión q1)
        p1, q1 = self.sample_pen_point(point_name="Punto 1 (Origen)", num_samples=100)

        # 3. Captura el Punto 2 (detrás en la línea: solo nos interesa la posición p2)
        p2, _ = self.sample_pen_point(point_name="Punto 2 (Z Axis)", num_samples=100)

        if p1 is not None and p2 is not None:
            self.T_ndi_base, self.T_base_ndi_inv = self.compute_base_transform(p1, q1, p2)
            self.flag_calibration_done = True
            print("=== CALIBRATION COMPLETED ===")

if __name__ == '__main__':

    rp.init_node('aurora_processing_node', anonymous=True)

    try:
        aurora = AuroraProcessing(calibrated=False)
        rp.spin()
    except rp.ROSInterruptException:
        print("Nodo finalizado.")
