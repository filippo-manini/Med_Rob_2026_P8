#!/usr/bin/env python3
import numpy as np
import sympy as sp # For Jacobian matrix
#from dynamixel_sdk_examples.msg import SetPosition
#from utils import quaternion_to_rotation_matrix


class PID_Controller:
    def __init__(self,
                 Kp=[0.01, 0.01, 0.01],
                 Kd=[0.0, 0.0, 0.0],
                 Ki=[0.0, 0.0, 0.0],
                 integral_limit=1.0,
                 debug=False):
        self.Kp = np.array(Kp, dtype=float)
        self.Kd = np.array(Kd, dtype=float)
        self.Ki = np.array(Ki, dtype=float)

        # allow scalar gains too (broadcast)
        if self.Kp.shape == ():
            self.Kp = np.full(3, float(self.Kp))
        if self.Ki.shape == ():
            self.Ki = np.full(3, float(self.Ki))
        if self.Kd.shape == ():
            self.Kd = np.full(3, float(self.Kd))

        self.integral_error = np.zeros(3, dtype=float)
        self.derivative_error = np.zeros(3, dtype=float)
        self.proportional_error = np.zeros(3, dtype=float)
        self.previous_position = None
        self.previous_error = np.zeros(3, dtype=float)
        self.initialized = False  # Flag to handle first actuation

        # Anti-windup limit (per-axis)
        self.integral_limit = np.full(3, float(integral_limit)) \
            if np.isscalar(integral_limit) else np.array(integral_limit, dtype=float)

        self.debug = debug
        self.use_pid = True

    def compute_control_action(self, target_position, current_position):
        target_position = np.array(target_position, dtype=float)
        current_position = np.array(current_position, dtype=float)

        if self.debug:
            print("target position is:", target_position)
            print("current position is:", current_position)

        self.proportional_error = target_position - current_position

        if not self.initialized:
            # Initialize previous states and return P-term + consistent ds=0.0
            self.previous_position = current_position.copy()
            self.previous_error = self.proportional_error.copy()
            self.initialized = True

            u = self.Kp * self.proportional_error
            ds = 0.0
            if self.debug:
                print("First step (initialized). P-only:", u, "ds:", ds)
            return u

        if self.debug:
            print("previous position is:", self.previous_position)
            print("current position is:", current_position)

        # distance since last call (used as spatial delta)
        ds = np.linalg.norm(current_position - self.previous_position)
        ds_eff = ds if ds > 1e-12 else 1e-12  # small epsilon to avoid division by zero

        if self.debug:
            print("ds:", ds)

        # integrate error scaled by ds
        self.integral_error += self.proportional_error * ds_eff

        # anti-windup: clamp integral per axis
        self.integral_error = np.clip(self.integral_error, -self.integral_limit, self.integral_limit)

        # derivative (error change per unit distance)
        self.derivative_error = (self.proportional_error - self.previous_error) / ds_eff

        # update state
        self.previous_error = self.proportional_error.copy()
        self.previous_position = current_position.copy()

        control_action = (
            self.Kp * self.proportional_error +
            self.Ki * self.integral_error +
            self.Kd * self.derivative_error
        )

        if self.debug:
            print("P:", self.Kp * self.proportional_error)
            print("I:", self.Ki * self.integral_error)
            print("D:", self.Kd * self.derivative_error)
            print("control_action:", control_action)

        return control_action


