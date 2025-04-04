#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

def compute_aik(position, orientation):
    """
    Function that calulates the PUMA analytical IK function.
    Should take a 3x1 position vector and a 3x3 rotation matrix,
    and return a list of joint positions.
    """
    
    print("\nReceived Position:")
    print(position)

    print("\nReceived Orientation (3x3 Rotation Matrix):")
    print(orientation)

    # PUMA Robot Parameters (meters)
    d1, a2, a3 = 0.150, 0.432, 0.432  # Given parameters
    px, py, pz = position[0], position[1], position[2]
    r1, r2, r3 = orientation[0], orientation[1], orientation[2]
    
    # Replace with the actual analytical IK computation
    # Insert your code here
    capitalD = (np.square(px) + np.square(py) + np.square(pz) - np.square(d1) - np.square(a2) - np.square(a3)) / (2 * a2 * a3)

    theta1 = np.pi + np.arctan2(py, px) + np.arctan2(-np.sqrt(np.square(px) + np.square(py) - np.square(d1)), d1)
    theta3 = np.arctan2(np.sqrt(1 - np.square(capitalD)), capitalD)
    theta2 = np.arctan2(pz, np.sqrt(np.square(px) + np.square(py) - np.square(d1))) - np.arctan2(a3 * np.sin(theta3), a2 + (a3 * np.cos(theta3)))

    if np.abs(r3[0]) != 1:
        theta4 = np.arctan2(r2[0], r1[0])
        theta5 = np.arctan2(-r3[0], np.sqrt(np.square(r1[0]) + np.square(r2[0])))
        theta6 = np.arctan2(r3[1], r3[2])
    elif r3[0] == -1:
        theta4 = 0
        theta5 = np.pi / 2
        theta6 = np.arctan2(r1[1], r2[1])
    else:
        theta4 = 0
        theta5 = -np.pi / 2
        theta6 = -np.arctan2(r1[1], r2[1])

    joint_positions = np.array([theta1, theta2, theta3, theta4, theta5, theta6])

    print("join_positions:")
    print(joint_positions)

    return joint_positions

def pose_callback(msg):
    """
    Callback function to handle incoming end-effector pose messages.
    You probably do not have to change this
    """
    # Extract position (3x1)
    position = np.array([msg.position.x, msg.position.y, msg.position.z])

    # Extract orientation (3x3 rotation matrix from quaternion)
    q = msg.orientation
    orientation = np.array([
        [1 - 2 * (q.y**2 + q.z**2), 2 * (q.x*q.y - q.z*q.w), 2 * (q.x*q.z + q.y*q.w)],
        [2 * (q.x*q.y + q.z*q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y*q.z - q.x*q.w)],
        [2 * (q.x*q.z - q.y*q.w), 2 * (q.y*q.z + q.x*q.w), 1 - 2 * (q.x**2 + q.y**2)]
    ])

    # Compute inverse kinematics
    joint_positions = compute_aik(position, orientation)

    # Publish joint states
    joint_msg = JointState()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = [f"joint{i+1}" for i in range(len(joint_positions))]
    joint_msg.position = joint_positions
    joint_pub.publish(joint_msg)

if __name__ == "__main__":
    rospy.init_node("ik_solver_node", anonymous=True)

    # Publisher: sends joint positions
    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    
    print("\nWaiting for /goal_pose")
    
    # Subscriber: listens to end-effector pose
    rospy.Subscriber("/goal_pose", Pose, pose_callback)

    rospy.spin()
