#!/usr/bin/env python3
import numpy as np
import casadi as ca
import scipy.linalg
import cvxpy as cp
import sympy as sp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import rospy as rp




import numpy as np
from math import sin, pi

class KalmanPredictor:
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise, measured_frequency):
        """
        initial_state: numpy array di shape (3,)
        initial_covariance: matrice P0, shape (3, 3)
        process_noise: matrice Q, shape (3, 3)
        measurement_noise: scalare R
        measured_frequency: frequenza del segnale osservato (Hz)
        """
        self.x = initial_state
        self.P = initial_covariance
        self.Q = process_noise
        self.R = measurement_noise
        self.f = measured_frequency
        self.t = 0  # tempo iniziale
        print("INITILIZED!!")

    def predict(self, delta_t):
        """
        Esegue la predizione dello stato al tempo t + delta_t.
        """
        self.t += delta_t
        period = 1 / self.f
        self.t %= period  # rimane nel range [0, T)

        F = np.eye(3)

        # Predizione del primo stato (mapse)
        if self.t < period / 2:
            mapse_pred = self.x[2] * sin(2 * pi * self.f * self.t) + self.x[1]
            F[0,2] = sin(2 * pi * self.f * self.t)
        else:
            mapse_pred = - 1 * self.x[2] * sin(2 * pi * self.f * self.t) + self.x[1]
            F[0,2] = -1 * sin(2 * pi * self.f * self.t)
        F[0,0] = 0
        F[0,1] = 1  # derivata rispetto all'offset

        # Stato previsto
        self.x = np.array([mapse_pred, self.x[1], self.x[2]])

        # Covarianza prevista
        self.P = F @ self.P @ F.T + self.Q

        return 
    
    def predict_no_update(self, delta_t):
        """
        Esegue la predizione dello stato al tempo t + delta_t.
        """
        t = self.t + delta_t
        period = 1 / self.f
        t %= period  # rimane nel range [0, T)

        

        # Predizione del primo stato (mapse)
        if t < period / 2:
            mapse_pred = self.x[2] * sin(2 * pi * self.f * t) + self.x[1]
            
        else:
            mapse_pred = -0.25 * self.x[2] * sin(2 * pi * self.f * t) + self.x[1]
            


        # Stato previsto
        x = np.array([mapse_pred, self.x[1], self.x[2]])

        # Covarianza prevista
        
        return mapse_pred
    
    def predict_helper(self, t):
        """
        Esegue la predizione dello stato al tempo t + delta_t.
        """
        
        period = 1 / self.f
        t %= period  # rimane nel range [0, T)

        

        # Predizione del primo stato (mapse)
        if t < period / 2:
            mapse_pred = self.x[2] * sin(2 * pi * self.f * t) + self.x[1]
            
        else:
            mapse_pred = -0.25 * self.x[2] * sin(2 * pi * self.f * t) + self.x[1]
            


        # Stato previsto
        x = np.array([mapse_pred, self.x[1], self.x[2]])

        # Covarianza prevista
        
        return mapse_pred

    def correct(self, observation):
        """
        Corregge la predizione usando l'osservazione attuale.
        """
        H = np.array([[1, 0, 0]])  # osserva solo il primo stato
        y = np.array([observation - self.x[0]])  # residuo

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        #print("KLMAN GAIN:          ",K)
        # Aggiornamento stato e covarianza
        self.x = self.x + K @ y
        #print("CORRECTED STATE", self.x)
        self.P = (np.eye(3) - K @ H) @ self.P

        return 
