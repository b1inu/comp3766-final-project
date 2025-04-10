#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import modern_robotics as mr
import numpy as np
import time

class NumericalIKNode(Node):
    def __init__(self):
        super().__init__(node_name='numerical_ik_node')
        self.get_logger().info("NIK node started...")
        
        # goal configuration of end-effector
        self.T_sd = np.array([
            [0, 0, 1, 0.5],
            [0, 1, 0, 0.1],
            [-1, 0, 0, 0.6],
            [0, 0, 0, 1]])
        
        # Screw axes in the space frame: 
        self.Slist = np.array([
            [0, 0, 1, 0, 0, 0],
            [0, 1, 0, -0.3, 0, 0],
            [0, 1, 0, -0.6, 0, 0],
            [0, 0, -1, 0, 0.6, 0],
            [0, 1, 0, -0.9, 0, 0],
            [1, 0, 0, 0, -0.9, 0]
        ]).T
        
        # Home configuration of the robot: all joint angles are zero
        self.M = np.array([
            [1, 0, 0, 0.9],
            [0, 1, 0, 0],
            [0, 0, 1, 1.2],
            [0, 0, 0, 1]
        ])
        
        self.theta_guess = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        
        self.solve_ik()
        
    def solve_ik(self):
        eomg = 1e-3
        ev = 1e-3
        max_iterations = 100
        theta = self.theta_guess.copy()
        
        start = time.time()
        
        try:
            for i in range(max_iterations):
                T_current = mr.FKinSpace(self.M, self.Slist, theta)
                V_b = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(T_current), self.T_sd)))
                err = np.linalg.norm(V_b[0:3]) > eomg or np.linalg.norm(V_b[3:6]) > ev
                if not err:
                    end = time.time()
                    self.get_logger().info(f"[SUCCESS!] IK took {i} iterations, and {end - start:.6f} seconds.")
                    self.get_logger().info(f"Joint Angles (rad): {theta}")
                    return
                J = mr.JacobianSpace(self.Slist, theta)
                V_s = mr.Adjoint(T_current) @ V_b
                theta += np.dot(np.linalg.pinv(J), V_s)
            
            end = time.time()
            self.get_logger().warn(f"[FAILURE!] IK failed to converge after {max_iterations} iterations.")
            self.get_logger().info(f"Final Joint Angles: {theta}")
            self.get_logger().info(f"Time taken is {end - start:.6f} seconds.")
        except Exception as e:
            self.get_logger().error(f"Error in IK solver: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = NumericalIKNode()
    # rclpy.spin_once(node, timeout_sec=1.0)
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()