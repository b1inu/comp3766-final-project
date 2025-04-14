#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('analytical_ik_node')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.pose_sub = self.create_subscription(Pose, '/goal_pose', self.pose_callback, 10)
        self.get_logger().info("IK Solver Node initialized. Waiting for /goal_pose...")

    # Compute the anayltical inverse kinematics (assumes a DH model)
    def compute_aik(self, position, orientation):
        # Robot link parameters
        d1, a2, a3 = 0.150, 0.432, 0.432
        px, py, pz = position[0], position[1], position[2]
        r1, r2, r3 = orientation[0], orientation[1], orientation[2]

        # Compute wrist position + joint angles
        capitalD = (px**2 + py**2 + pz**2 - d1**2 - a2**2 - a3**2) / (2 * a2 * a3)

        theta1 = np.pi + np.arctan2(py, px) + np.arctan2(-np.sqrt(px**2 + py**2 - d1**2), d1)
        theta3 = np.arctan2(np.sqrt(1 - capitalD**2), capitalD)
        theta2 = np.arctan2(pz, np.sqrt(px**2 + py**2 - d1**2)) - np.arctan2(a3 * np.sin(theta3), a2 + a3 * np.cos(theta3))

        # Compute last 3 joints from rotation matrix
        if abs(r3[0]) != 1:
            theta4 = np.arctan2(r2[0], r1[0])
            theta5 = np.arctan2(-r3[0], np.sqrt(r1[0]**2 + r2[0]**2))
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
        self.get_logger().info(f"Joint Positions: {joint_positions}")
        return joint_positions

    def pose_callback(self, msg):
        position = np.array([msg.position.x, msg.position.y, msg.position.z])
        
        # Convert quaternion to rotation matrix
        q = msg.orientation
        orientation = np.array([
            [1 - 2 * (q.y**2 + q.z**2),     2 * (q.x*q.y - q.z*q.w),     2 * (q.x*q.z + q.y*q.w)],
            [2 * (q.x*q.y + q.z*q.w),     1 - 2 * (q.x**2 + q.z**2),     2 * (q.y*q.z - q.x*q.w)],
            [2 * (q.x*q.z - q.y*q.w),       2 * (q.y*q.z + q.x*q.w),     1 - 2 * (q.x**2 + q.y**2)]
        ])

        joint_positions = self.compute_aik(position, orientation)

        # Publish joint angles
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [f"joint{i+1}" for i in range(len(joint_positions))]
        joint_msg.position = joint_positions.tolist()
        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
