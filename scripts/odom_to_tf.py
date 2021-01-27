#!/usr/bin/env python

import rospy

import tf2_ros
from nav_msgs.msg import Odometry
import geometry_msgs.msg

import sys

class OdomToTF:
    def __init__(self, name):
        # Initialize node
        rospy.init_node(name);

    # Odometry subscriber callback
    def onOdom(self, msg):
        # Create the message to be published
        tf_msg_out = geometry_msgs.msg.TransformStamped()
        
        # Set header info
        tf_msg_out.header = msg.header
        tf_msg_out.child_frame_id = msg.child_frame_id

        tf_msg_out.transform.translation.x = msg.pose.pose.position.x
        tf_msg_out.transform.translation.y = msg.pose.pose.position.y
        tf_msg_out.transform.translation.z = msg.pose.pose.position.z
        tf_msg_out.transform.rotation = msg.pose.pose.orientation

        # Send it!
        self.tf_broadcaster.sendTransform(tf_msg_out)

    def init(self):
        # Fetch parameter(s)
        if rospy.has_param("~odom_topic"): # "~" Indicates it's a private topic (see ros parameter-server docs)
            self.sub_topic = rospy.get_param("~odom_topic")
        else:
            rospy.logerr("[OdomToTF]: Missing '~odom_topic' parameter")
            return -1

        # Create odometry topic subscriber
        self.groundtruth_sub = rospy.Subscriber(self.sub_topic, Odometry, self.onOdom)

        # Create tf broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        return 0

    def run(self):
        # Spin the node (start all the subscribers)
        rospy.loginfo("[OdomToTF]: Spinning node!")
        rospy.spin()

if __name__ == '__main__':

    # Grab node name arg
    if len(sys.argv) < 2:
        rospy.logerr("[OdomToTF]: Missing node name parameter")
    odom_to_tf = OdomToTF(sys.argv[1])

    if odom_to_tf.init() == 0:
        # Run the node
        try:
            odom_to_tf.run()
        except (KeyboardInterrupt):
            rospy.logerr("[OdomToTF]: Killing node!")
    else:
        rospy.logerr("[OdomToTF]: Initialization failed, killing node!")
