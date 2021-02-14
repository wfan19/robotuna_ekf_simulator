#!/usr/bin/env python3

from controller_factory import ControllerFactory
from controller_base import Controller

from pidff import PIDFFParams
from controller_pid_att import AttCtrlParams

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

import numpy as np

import sys

class ControllerNode:
    def __init__(self, name: str):
        self.node_name = name
        self.throttle_cmd = 0
        self.odom_ref = Odometry()
        self.odom_cur = Odometry()

        self.odom_ref.pose.pose.orientation.w = 1
        self.odom_cur.pose.pose.orientation.w = 1

    def run(self):
        rospy.init_node(self.node_name)

        # Controller params
        # TODO: Get parameters from rospy. Write a yaml-struct to pidff converter
        bodyrate_kP = np.diag([50, 50, 0.3])
        bodyrate_kI = np.diag([0.05, 0.05, 0.05])
        bodyrate_kD = np.diag([1, 1, 0.3])
        bodyrate_i_min = -0.01
        bodyrate_i_max = 0.01

        bodyrate_params = PIDFFParams(kP=bodyrate_kP, kI=bodyrate_kI, kD=bodyrate_kD, i_min = bodyrate_i_min, i_max = bodyrate_i_max)
        attitude_params = AttCtrlParams(kP=0.6, yaw_weight=0.3)

        # Create controller factory to generate the controller
        controller_factory = ControllerFactory(bodyrate_params=bodyrate_params, attitude_params=attitude_params)

        # Create the controller object from the controller factory
        self.controller = controller_factory.create_controller("Attitude", rospy.get_time())

        # Initialize ROS publishers and subscribers
        self.odom_ref_sub = rospy.Subscriber("/Kwad/cmd_odom", Odometry, self.onOdomRef)
        self.odom_cur_sub = rospy.Subscriber("/Kwad/odometry/ground_truth/map", Odometry, self.onOdomCur)
        self.throttle_sub = rospy.Subscriber("/Kwad/cmd_throttle", Float64, self.onThrottle)
        
        self.control_pub = rospy.Publisher("/Kwad/group_props_controller/command", Float64MultiArray, queue_size=1)

        # Create controller timer
        # The timer calls the controller callback function (self.control) periodically
        controller_period = rospy.Duration(0.01) # 100hz
        controller_timer = rospy.Timer(controller_period, self.control)
        
        # Spin the node
        # This primes all the ROS callbacks
        # Blocks till program ends
        rospy.loginfo("[ControllerNode]: Spinning!")
        rospy.spin()

    def control(self, timer_event: rospy.timer.TimerEvent):
            t_now = timer_event.current_real.to_sec()
            # Calculate control value
            control_vals_msg = self.controller.control(t_now, self.throttle_cmd, self.odom_ref, self.odom_cur)
            
            # Publish control messages
            self.control_pub.publish(control_vals_msg)

    # Save new Odometry references
    def onOdomRef(self, odom_ref_msg: Odometry):
        self.odom_ref = odom_ref_msg

    # Save new Odometry measurement
    def onOdomCur(self,odom_cur_msg: Odometry):
        self.odom_cur = odom_cur_msg

    # Save new throttle setpoint
    def onThrottle(self, throttle_cmd: Float64):
        self.throttle_cmd = throttle_cmd.data

if __name__ == '__main__':
    # Grab node name arg
    if len(sys.argv) < 2:
        rospy.logerr("[ControllerNode]: Missing node name parameter")
    controllerNode = ControllerNode(sys.argv[1])

    # Run the node
    try:
        controllerNode.run()
    except (KeyboardInterrupt):
        rospy.logerr("[ControllerNode]: Killing node!")

    else:
        rospy.logerr("[ControllerNode]: Initialization failed, killing node!")
