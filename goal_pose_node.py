#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix

# Global constants for the goal pose (students should modify these values)
# GOAL_POSITION = np.array([-0.15, 0.429+0.429, 0.0]) # Modify these values 
GOAL_POSITION = np.array([-0.25, 0.50, 0.0]) # Modify these values 
GOAL_ROTATION_MATRIX = np.array([
    [-1.0, 0.0, 0.0],  # Modify these values
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0]
])

# List of position matricies for testing/comparison
pos = [[-0.25, 0.50, 0.15],
       [0.25, -0.50, 0.0],
       [-0.75, 0.25, 0.0],
       [0.50, 0.50, 0.0],
       [-0.25, -0.25, 0.0],
       [0.30, 0.30, 0.30],
       [-0.75, 0.0, 0.20],
       [0.55, 0.15, -0.25],
       [0.0, 0.50, -0.20],
       [-0.30, -0.10, 0.50]]

# List of orientation matricies for testing/comparison
rot = [[[-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0]],
       [[1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]],
       [[0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0]],
       [[0.0, 0.0, -1.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0]]]

GOAL_POSITIONS = np.array(pos)
GOAL_ROTATION_MATRICIES = np.array(rot)

def publish_goal_pose():
    rospy.init_node("goal_pose_publisher", anonymous=True)
    pose_pub = rospy.Publisher("/goal_pose", Pose, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Convert rotation matrix to quaternion
    rotation_matrix_4x4 = np.eye(4)
    rotation_matrix_4x4[:3, :3] = GOAL_ROTATION_MATRICIES[0]
    quaternion = quaternion_from_matrix(rotation_matrix_4x4)

    goal_position = GOAL_POSITIONS[0]

    # Define the goal pose
    goal_pose = Pose()
    goal_pose.position.x = goal_position[0]
    goal_pose.position.y = goal_position[1]
    goal_pose.position.z = goal_position[2]

    goal_pose.orientation.x = quaternion[0]
    goal_pose.orientation.y = quaternion[1]
    goal_pose.orientation.z = quaternion[2]
    goal_pose.orientation.w = quaternion[3]

    rospy.loginfo("Publishing goal pose...")
    rospy.loginfo(f"Goal Position: {goal_position}")
    rospy.loginfo(f"Goal Rotation Matrix:\n{GOAL_ROTATION_MATRICIES[0]}")
    rospy.loginfo(f"Converted Quaternion: {quaternion}")

    while not rospy.is_shutdown():
        pose_pub.publish(goal_pose)
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_goal_pose()
    except rospy.ROSInterruptException:
        pass
