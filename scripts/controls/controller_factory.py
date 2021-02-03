#!/usr/bin/env python3

from pidff import *

from controller_base import Controller
#from controller_pid_pos import Controller
from controller_pid_att import *
from controller_pid_bodyrate import *

class ControllerFactory:

    # TODO: This class currently is not structured in a very scalable way...
    #       A better way to structure this whole thing would be if the controller
    #       classes have a constructor where they use dependent *controllers* 
    #       instead of dependent controller *configs*. And then I can just create
    #       controllers here...

    def __init__(self, 
        position_params: PIDFFParams=None,
        velocity_params: PIDFFParams=None,
        attitude_params: AttCtrlParams=None,
        bodyrate_params: PIDFFParams=None
    ):
        #TODO: Once we separate out velocity control to a separate class, these two dicts
        #      could be combined into a set of objects, perhaps?
        #      Then we can have the objects handle the logic for checking dependent config objects
        self.configs = {
            "Position": position_params,
            "Velocity": velocity_params,
            "Attitude": attitude_params,
            "Bodyrate": bodyrate_params
        }

        # TODO: This can be combined with self.configs? See above
        self.factory_methods = {
            "Bodyrate": self.create_pid_bodyrate,
            "Attitude": self.create_pid_attitude
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
    
    def create_pid_attitude(self, time):
        if self.configs["Attitude"] != None and self.configs["Bodyrate"] != None:
            controller = ControllerPIDAtt(time, self.configs["Attitude"], self.configs["Bodyrate"])
            return controller
        else:
            print("Bodyrate or Attitude configuration missing!")
            return -1


    # other controller constructors can be implemented later