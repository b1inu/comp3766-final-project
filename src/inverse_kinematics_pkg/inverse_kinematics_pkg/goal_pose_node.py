#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose
from transforms3d.quaternions import mat2quat

GOAL_POSITION = [0.25, 0.5, 0.0]
GOAL_ROTATION_MATRIX = np.array([
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0]
])

class GoalPosePublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_node')
        self.publisher_ = self.create_publisher(Pose, '/goal_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_goal_pose)
        
        quaternion = mat2quat(GOAL_ROTATION_MATRIX)

        self.get_logger().info("GoalPosePublisher initialized and ready.")

        self.goal_pose = Pose()
        self.goal_pose.position.x = GOAL_POSITION[0]
        self.goal_pose.position.y = GOAL_POSITION[1]
        self.goal_pose.position.z = GOAL_POSITION[2]
        self.goal_pose.orientation.x = quaternion[1]  # x
        self.goal_pose.orientation.y = quaternion[2]  # y
        self.goal_pose.orientation.z = quaternion[3]  # z
        self.goal_pose.orientation.w = quaternion[0]  # w
        
        # Log some info for debugging
        self.get_logger().info("Publishing goal pose...")
        self.get_logger().info(f"Goal Position: {GOAL_POSITION}")
        self.get_logger().info(f"Goal Rotation Matrix:\n{GOAL_ROTATION_MATRIX}")
        self.get_logger().info(f"Converted Quaternion: {quaternion}")

    def publish_goal_pose(self):
        # Publish the goal pose message
        self.publisher_.publish(self.goal_pose)

def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
