# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        self.dt = params.dt
        self.dim_state = params.dim_state
        self.q = params.q

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############

        return(np.matrix([[1, 0, 0, self.dt, 0, 0],[0, 1, 0, 0, self.dt, 0], [0, 0, 1, 0, 0, self.dt], [0, 0, 0, 1, 0, 0],[0, 0, 0, 0, 1, 0],[0, 0, 0, 0, 0, 1]]))
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        
        Q1 = self.dt * self.q
        halfQ2 = (1/2) * self.dt**2 * self.q
        thirdQ3 = (1/3) * self.dt**3 * self.q
        return np.matrix([[thirdQ3, 0, 0, halfQ2, 0, 0],[0, thirdQ3, 0, 0, halfQ2, 0],[0, 0, 0, 0, 0, 0],[halfQ2, 0, 0, Q1, 0, 0],[0, halfQ2, 0, 0, Q1, 0],[0, 0, 0, 0, 0, 0]])
        
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        F = self.F()
        x = F * track.x # state prediction
        P = F * track.P * F.transpose() + self.Q() # covariance prediction
        track.set_x(x)
        track.set_P(P)
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        
        K = track.P * meas.sensor.get_H(track.x).transpose() * np.linalg.inv(self.S(track, meas)) # Kalman gain
        x = track.x + K * self.gamma(track, meas) # state update
        I = np.identity(self.dim_state)
        P = (I - K * meas.sensor.get_H(track.x)) * track.P # covariance update
        track.set_x(x)
        track.set_P(P)  
        ############
        # END student code
        ############

        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        return  (meas.z - meas.sensor.get_hx(track.x)) # residual
        
        ############
        # END student code
        ############ 

    def S(self, track, meas):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        return (meas.sensor.get_H(track.x) * track.P * meas.sensor.get_H(track.x).transpose() + meas.R)  # covariance of residual
        
        ############
        # END student code
        ############ 