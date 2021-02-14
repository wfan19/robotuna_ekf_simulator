#!/usr/bin/env python3

# A node for converting xyz-order Euler angle orientation targets to target odometry messages

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

from scipy.spatial.transform import Rotation

import sys

class EulerToOdom:
    def __init__(self, name):
        # Initialize node
        rospy.init_node(name);

    # Odometry subscriber callback
    def onEuler(self, msg: Vector3):
        v_input_euler = [msg.x, msg.y, msg.z]
        q_input = Rotation.from_euler("xyz", v_input_euler, degrees=self.degrees).as_quat()
        
        odom_out = Odometry()
        odom_out.pose.pose.orientation.x = q_input[0]
        odom_out.pose.pose.orientation.y = q_input[1]
        odom_out.pose.pose.orientation.z = q_input[2]
        odom_out.pose.pose.orientation.w = q_input[3]
        self.odom_pub.publish(odom_out)

    def init(self):
        # Fetch parameter(s)
        if rospy.has_param("~odom_topic"): # "~" Indicates it's a private topic (see ros parameter-server docs)
            self.pub_topic = rospy.get_param("~odom_topic")
        else:
            rospy.logerr("[EulerToOdom]: Missing '~odom_topic' parameter")
            return -1

        if rospy.has_param("~euler_topic"):
            self.sub_topic = rospy.get_param("~euler_topic")
        else:
            rospy.logerr("[EulerToOdom]: Missing '~euler_topic' parameter")
            return -1

        self.degrees = rospy.get_param("~degrees", True)

        # Create odometry topic subscriber
        self.euler_sub = rospy.Subscriber(self.sub_topic, Vector3, self.onEuler)
        self.odom_pub = rospy.Publisher(self.pub_topic, Odometry, queue_size=1)
        return 0

    def run(self):
        # Spin the node (start all the subscribers)
        rospy.loginfo("[EulerToOdom]: Spinning node!")
        rospy.spin()

if __name__ == '__main__':

    # Grab node name arg
    if len(sys.argv) < 2:
        rospy.logerr("[EulerToOdom]: Missing node name parameter")
    euler_to_odom = EulerToOdom(sys.argv[1])

    if euler_to_odom.init() == 0:
        # Run the node
        try:
            euler_to_odom.run()
        except (KeyboardInterrupt):
            rospy.logerr("[EulerToOdom]: Killing node!")
    else:
        rospy.logerr("[EulerToOdom]: Initialization failed, killing node!")
