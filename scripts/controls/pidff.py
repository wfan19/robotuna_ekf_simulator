#!/usr/bin/env python3
import numpy as np

class PIDFFParams:
    kP = 1
    kI = 0
    kD = 0
    kFF = 0
    i_min = -1
    i_max = 1

    def __init__(self, kP = 1, kI = 0, kD = 0, kFF = 0, i_min = -1, i_max = 1):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kFF = kFF
        self.i_max = i_max
        self.i_min = i_min

class PIDFF:

    def __init__(self, params: PIDFFParams = PIDFFParams(), time = 0):
        self.params = params
        self.last_time = time
        self.last_error = 0
        self.i_term = 0

    def update(self, time, reference, measurement):
        error = reference - measurement
        dt = time - self.last_time

        # Calculate p, d, and ff terms
        p_term = np.dot(self.params.kP, error)
        d_term = np.dot(self.params.kD, (error - self.last_error)) / dt
        ff_term = np.dot(self.params.kFF, reference)

        # Calculate and clamp i term
        self.i_term = self.i_term + np.dot(self.params.kI, error) * dt

        self.i_term = np.clip(self.i_term, self.params.i_min, self.params.i_max)

        # Fuse terms
        control_val = p_term + self.i_term + d_term + ff_term

        # Update storage
        self.last_time = time
        self.last_error = error
        
        return control_val
    
    def reset(self, time):
        self.last_time = time
        self.last_error = 0
        self.i_term = 0
    