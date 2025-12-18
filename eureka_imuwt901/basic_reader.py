#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import Vector3
import serial
import struct
import time

class WT901Node(Node):
    def __init__(self):
        super().__init__('wt901_node')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        self.g = 9.8
        self.PI = 3.1415926

        self._setup_device()

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.temp_pub = self.create_publisher(Temperature, 'imu/temp', 10)

        self.create_timer(0.01, self.read_imu)  # 100 Hz
        self.create_timer(0.5, self.read_misc)  # 2 Hz

    def _setup_device(self):
        self.ser.write(b'\xFF\xAA\x03\x00\x00')  # stop stream
        time.sleep(0.05)

    def _read_register(self, reg):
        self.ser.write(bytes([0xFF, 0xAA, 0x27, reg, 0x00]))
        start = time.time()
        while time.time() - start < 0.1:
            if self.ser.read(1) == b'\x55':
                if self.ser.read(1) == b'\x71':
                    self.ser.read(2)  # discard register info
                    data = self.ser.read(16)
                    if len(data) == 16:
                        return data
        return None

    def _parse_2s16(self, lo, hi):
        return int.from_bytes([hi, lo], byteorder='big', signed=True)

    def read_imu(self):
        raw_imu = self._read_register(0x34)  # AX -> GZ
        raw_angle = self._read_register(0x3D)  # Roll -> Yaw

        if not raw_imu or len(raw_imu) < 12 or not raw_angle or len(raw_angle) < 6:
            self.get_logger().warn("IMU data read failed or incomplete")
            return

        try:
            # Parse AX–GZ
            accel_gyro = [self._parse_2s16(raw_imu[i], raw_imu[i+1]) for i in range(0, 12, 2)]
            ax, ay, az = [v / 32768 * 16 * self.g for v in accel_gyro[0:3]]
            wx, wy, wz = [v / 32768 * 2000 / 180 * self.PI for v in accel_gyro[3:6]]

            # Parse Roll, Pitch, Yaw
            angle_vals = [self._parse_2s16(raw_angle[i], raw_angle[i+1]) for i in range(0, 6, 2)]
            roll, pitch, yaw = [v / 32768 * self.PI for v in angle_vals]

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.linear_acceleration = Vector3(x=ax, y=ay, z=az)
            imu_msg.angular_velocity = Vector3(x=wx, y=wy, z=wz)
            imu_msg.orientation_covariance[0] = -1  # unknown orientation

            self.imu_pub.publish(imu_msg)

        except IndexError:
            self.get_logger().warn("Parsing error in IMU data")


        try:
            vals = [self._parse_2s16(raw_imu[i], raw_imu[i+1]) for i in range(0, 18, 2)]
            ax, ay, az = [v / 32768 * 16 * self.g for v in vals[0:3]]
            wx, wy, wz = [v / 32768 * 2000 / 180 * self.PI for v in vals[3:6]]
            roll, pitch, yaw = [v / 32768 * self.PI for v in vals[6:9]]

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            imu_msg.linear_acceleration = Vector3(x=ax, y=ay, z=az)
            imu_msg.angular_velocity = Vector3(x=wx, y=wy, z=wz)
            imu_msg.orientation_covariance[0] = -1  # unknown orientation

            self.imu_pub.publish(imu_msg)
        except IndexError:
            self.get_logger().warn("IMU register response too short")

    def read_misc(self):
        raw_mag = self._read_register(0x3A)
        if raw_mag:
            hx, hy, hz = [self._parse_2s16(raw_mag[i], raw_mag[i+1]) for i in range(0, 6, 2)]
            mag_msg = MagneticField()
            mag_msg.header.stamp = self.get_clock().now().to_msg()
            mag_msg.header.frame_id = 'imu_link'
            mag_msg.magnetic_field.x = hx * 1e-6  # uT
            mag_msg.magnetic_field.y = hy * 1e-6
            mag_msg.magnetic_field.z = hz * 1e-6
            self.mag_pub.publish(mag_msg)
        else:
            self.get_logger().warn("Magnetometer read failed")

        raw_temp = self._read_register(0x40)
        if raw_temp:
            temp = self._parse_2s16(raw_temp[0], raw_temp[1]) / 100
            temp_msg = Temperature()
            temp_msg.header.stamp = self.get_clock().now().to_msg()
            temp_msg.header.frame_id = 'imu_link'
            temp_msg.temperature = temp
            self.temp_pub.publish(temp_msg)
        else:
            self.get_logger().warn("Temperature read failed")

def main():
    rclpy.init()
    node = WT901Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
