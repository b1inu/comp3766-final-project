#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import modern_robotics as mr
import numpy as np
import transforms3d.quaternions as tq
import time

class NumericalIKNode(Node):
    def __init__(self):
        super().__init__('numerical_ik_node')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.pose_sub = self.create_subscription(Pose, '/goal_pose', self.pose_callback, 10)
        self.get_logger().info("Numerical IK Node ready and waiting for /goal_pose...")

        # Screw axes
        self.Slist = np.array([
            [0, 0, 1,     0,       0,     0],
            [0, 1, 0, -0.67,      0,     0],
            [0, 1, 0, -1.102,     0,     0],
            [0, 0, 1,     0, -1.514,     0],
            [0, 1, 0, -1.514,     0,     0],
            [1, 0, 0,     0, -1.514,     0]
        ]).T

        # Home configuration
        self.M = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 1.514],
            [0, 0, 0, 1]
        ])

        # Initial guess (using our analytical IK results)
        self.theta_guess = np.array([2.62456958, -0.03878836, 1.34310751, 3.14159265, -0., 3.14159265])

    # Wrap angles to be within [-pi, pi]
    def wrap_angles(self, thetas):
        return (thetas + np.pi) % (2 * np.pi) - np.pi

    # Callback for receiving the goal pose
    def pose_callback(self, msg):
        position = np.array([[msg.position.x], [msg.position.y], [msg.position.z]])
        q = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        rotation_matrix = tq.quat2mat(q)

        T_sd = np.vstack((np.hstack((rotation_matrix, position)), [0, 0, 0, 1])) # build the transformation matrix
        self.solve_ik(T_sd)

    # Solve the inverse kinematics using modern robotics
    def solve_ik(self, T_sd):
        eomg = 1e-3
        ev = 1e-3
        max_iterations = 100
        theta = self.theta_guess.copy()

        start = time.time()

        for i in range(max_iterations):
            T_current = mr.FKinSpace(self.M, self.Slist, theta)
            V_b = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(T_current), T_sd)))
            err = np.linalg.norm(V_b[0:3]) > eomg or np.linalg.norm(V_b[3:6]) > ev
            if not err:
                end = time.time()
                self.get_logger().info(f"[SUCCESS] IK solved in {i} iterations, time: {end - start:.6f}s")
                self.publish_joint_states(theta)
                return
            J = mr.JacobianSpace(self.Slist, theta)
            V_s = mr.Adjoint(T_current) @ V_b
            theta += np.dot(np.linalg.pinv(J), V_s)
            
            T_actual = mr.FKinSpace(self.M, self.Slist, theta)

            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # Uncomment for debugging
            # self.get_logger().info("=== Forward Kinematics Debug ===")
            # self.get_logger().info(f"T_actual:\n{np.round(T_actual, 3)}")
            # self.get_logger().info(f"T_goal:\n{np.round(self.T_sd, 3)}")
            # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            
            diff = np.abs(T_actual - self.T_sd)
            self.get_logger().info(f"Difference:\n{np.round(diff, 4)}")

        end = time.time()
        self.get_logger().warn(f"[FAILURE] IK did not converge in {max_iterations} iterations.")
        self.get_logger().info(f"Final Joint Angles: {theta}")
        self.get_logger().info(f"Time taken: {end - start:.6f}s")

    # Publish the joint angles
    def publish_joint_states(self, theta):
        wrapped = self.wrap_angles(theta)
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [f"joint{i+1}" for i in range(6)]
        joint_msg.position = wrapped.tolist()
        self.joint_pub.publish(joint_msg)
        self.get_logger().info(f"Published Joint Angles: {wrapped}")

def main(args=None):
    rclpy.init(args=args)
    node = NumericalIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
