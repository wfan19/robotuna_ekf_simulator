#!/usr/bin/env python3
import numpy as np

class PIDFFGains:
    kP = 1
    kI = 0
    kD = 0
    kFF = 0
    i_min = -1
    i_max = 1

    def __init__(self, kP, kI = 0, kD = 0, kFF = 0, i_min = -1, i_max = 1):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kFF = kFF
        self.i_max = i_max
        self.i_min = i_min

class PIDFF:

    def __init__(self, gains: PIDFFGains = PIDFFGains(1), time = 0):
        self.gains = gains
        self.last_time = time
        self.last_error = 0
        self.i_term = 0

    def update(self, time, reference, measurement):
        error = reference - measurement
        dt = time - self.last_time

        # Calculate p, d, and ff terms
        p_term = self.gains.kP * error
        d_term = self.gains.kD * (error - self.last_error) / dt
        ff_term = self.gains.kFF * reference

        # Calculate and clamp i term
        self.i_term = self.i_term + (self.gains.kI * error) * dt

        self.i_term = np.clip(self.i_term, self.gains.i_min, self.gains.i_max)

        # Fuse terms
        control_val = p_term + self.i_term + d_term + ff_term

        # Update storage
        self.last_time = time
        self.last_error = error
        
        return control_val
    