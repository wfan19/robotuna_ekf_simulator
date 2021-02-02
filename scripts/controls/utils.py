#!/usr/bin/env python3

import numpy as np

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3

def ros_pose_to_np_arrays(pose: Pose):
    # [x; y; z] position vector
    position = np.array([[pose.position.x], [pose.position.y], [pose.position.z]])
    
    # [x; y; z; w] heading quaternion vector
    # Note: Here we keep W (the real component) in the last place. This is the ordering that SciPy uses
    # Also note: SciPy only takes 1d vectors and not column vectors. Thus this is returns a 1d vector
    heading = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    return (position, heading)

def ros_vector3_to_np_array(vector: Vector3):
    # [x; y; z] 3 element vector
    return np.array([[vector.x], [vector.y], [vector.z]])

def np_arrays_to_ros_pose(position: np.array, heading: np.array):
    pose_out = Pose()

    pose_out.position.x = position[0]
    pose_out.position.y = position[1]
    pose_out.position.z = position[2]

    pose_out.orientation.w = heading[0]
    pose_out.orientation.x = heading[1]
    pose_out.orientation.y = heading[2]
    pose_out_out.orientation.z = heading[3]

    return pose_out