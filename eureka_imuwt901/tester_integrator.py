#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from tf_transformations import euler_from_quaternion

class ImuIntegrator(Node):
    def __init__(self):
        super().__init__('imu_integrator')
        self.subscription = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.prev_time = None
        self.velocity = np.zeros(3)
        self.position = np.zeros(3)

    def imu_callback(self, msg):
        # Time delta
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.prev_time is None:
            self.prev_time = t
            return
        dt = t - self.prev_time
        self.prev_time = t

        # Get linear acceleration
        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Optional: remove gravity here if it's not already removed
        # acc = self.remove_gravity(acc, msg.orientation)

        # Integrate to get velocity and position
        self.velocity += acc * dt
        self.position += self.velocity * dt

        # Get orientation (quaternion to euler)
        q = msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Print current state
        self.get_logger().info(
            f"Position: {self.position.round(3)} | Orientation (rpy): {np.degrees(euler).round(1)}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ImuIntegrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()