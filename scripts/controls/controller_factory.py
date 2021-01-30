#!/usr/bin/env python3

from pidff import *

from controller_base import Controller
from controller_pid_pos import Controller
from controller_pid_att import *
from controller_pid_bodyrate import *

class ControllerFactory:

    def __init__(self, 
        position_params: PIDFFParams=None,
        velocity_params: PIDFFParams=None,
        attitude_params: AttCtrlParams=None,
        bodyrate_params: PIDFFParams=None
    ):
        self.configs = {
            "Position": position_params,
            "Velocity": velocity_params,
            "Attitude": attitude_params,
            "Bodyrate": bodyrate_params
        }

        # TODO: This can be combined with self.configs?
        self.factory_methods = {
            "Bodyrate": self.create_pid_bodyrate
        }

    def create_controller(self, controller_type, time):        

        controller = self.factory_methods[controller_type](time)

        return controller

    def create_pid_bodyrate(self, time):
        if self.configs["Bodyrate"] != None:
            controller = ControllerPIDBodyrate(time, self.configs["Bodyrate"])
            return controller
        else:
            print("Body rate configuration missing!")
            return -1

    # other controller constructors can be implemented later