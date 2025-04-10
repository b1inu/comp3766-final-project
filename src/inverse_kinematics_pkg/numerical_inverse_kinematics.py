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
        
        R = np.array([
            [1.0, 0.0, 0.0],
            [ 0.0, 1.0, 0.0],
            [ 0.0,  0.0, 1.0]
        ])
        d = np.array([[0.2], [0.2], [1.0]])
        self.T = np.vstack((np.hstack((R, d)), [0, 0, 0, 1]))
    
        # Screw axes
        self.Slist = np.array([
            [0, 0, 1, 0, 0, 0],
            [0, 1, 0, -0.3, 0, 0],
            [0, 1, 0, -0.6, 0, 0],
            [0, 0, -1, 0, 0.6, 0],
            [0, 1, 0, -0.9, 0, 0],
            [1, 0, 0, 0, -0.9, 0]
        ]).T
        
        # Home config 
        self.M = np.array([
            [1, 0, 0, 0.9],
            [0, 1, 0, 0],
            [0, 0, 1, 1.2],
            [0, 0, 0, 1]
        ])
        
        # ...
        self.theta_guess = np.array([0, 0, 0, 0, 0, 0])
        
        # # debugging w/ random tests giving outrageousness
        # self.test_cases = [
        #     np.array([[0, 0, 1, 0.5], [0, 1, 0, 0.1], [-1, 0, 0, 0.6], [0, 0, 0, 1]]),
        #     np.array([[0, -1, 0, 0.3], [1, 0, 0, 0.2], [0, 0, 1, 1.0], [0, 0, 0, 1]]),
        #     np.array([[1, 0, 0, 1.1], [0, 1, 0, -0.1], [0, 0, 1, 1.3], [0, 0, 0, 1]]),
        #     np.array([[0, 1, 0, -0.1], [-1, 0, 0, 0.4], [0, 0, 1, 1.1], [0, 0, 0, 1]]),
        #     np.array([[0, 0, 1, 1.2], [0, 1, 0, 0.0], [-1, 0, 0, 1.5], [0, 0, 0, 1]])
        # ]
    
        # self.run_tests()
        self.solve_ik()

    # wrapping angles within pi to reduce such outrageousness        
    def wrap_angles(self, thetalist):
        return (thetalist + np.pi) % (2 * np.pi) - np.pi
        
    def solve_ik(self):
        eomg = 1e-3
        ev = 1e-3

        
        start = time.time()
        thetalist, success = mr.IKinSpace(
            self.Slist, self.M, self.T, self.theta_guess, eomg, ev)
        end = time.time()
        
        # Trynan debug ts with fkin
        print("actual =\n", mr.FKinSpace(self.M, self.Slist, thetalist))
        print("goal   =\n", self.T)
        
        if success:
            self.get_logger().info(f"[SUCCESS] Numerical IK solved in {end - start:.6f} seconds.")
        else:
            self.get_logger().warn(f"[FAILURE] Numerical IK did not converge.")

        wrapped = self.wrap_angles(thetalist)
        self.get_logger().info(f"Joint angles in radians: {thetalist}")
        self.get_logger().info(f"Wrapped angles in radians: {wrapped}")
        self.get_logger().info(f"Time taken: {end - start:.6f} seconds.")

    # def run_tests(self):
    #     eomg = 1e-3
    #     ev = 1e-3

    #     for i, T in enumerate(self.test_cases):
    #         self.get_logger().info(f"\n[Test Case {i+1}]")
    #         start = time.time()
    #         thetalist, success = mr.IKinSpace(self.Slist, self.M, T, self.theta_guess, eomg, ev)
    #         end = time.time()

    #         if success:
    #             self.get_logger().info(f"[SUCCESS] IK solved in {end - start:.6f} seconds.")
    #         else:
    #             self.get_logger().warn(f"[FAILURE] IK did not converge.")

    #         wrapped = self.wrap_angles(thetalist)
    #         self.get_logger().info(f"Joint Angles (rad): {thetalist}")
    #         self.get_logger().info(f"Wrapped Angles (rad): {wrapped}")
    #         self.get_logger().info(f"Time taken: {end - start:.6f} seconds.")

def main(args=None):
    rclpy.init(args=args)
    node = NumericalIKNode()
    # rclpy.spin_once(node, timeouTec=1.0)
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()