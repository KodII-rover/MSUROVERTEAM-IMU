#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import serial
import glob
import time
import struct
import math

DEBUG = True

class WT901Node(Node):
    def __init__(self):
        super().__init__('wt901_node')

        self.declare_parameter("orientation_covariance", [0.0479, 0.0, 0.0, 0.0, 0.0207, 0.0, 0.0, 0.0, 0.0041])
        self.declare_parameter("linear_acceleration_covariance", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("angular_velocity_covariance", [0.0663, 0.0, 0.0, 0.0, 0.1453, 0.0, 0.0, 0.0, 0.0378])

        self.orientation_covariance = self.get_parameter("orientation_covariance").value
        self.linear_acceleration_covariance = self.get_parameter("linear_acceleration_covariance").value
        self.angular_velocity_covariance = self.get_parameter("angular_velocity_covariance").value

      #  ports = glob.glob('/dev/serial/by-id/usb-1a86*')
      #  if not ports:
      #      self.get_logger().error("IMU device not found! Check connections.")
      #      exit(1)
    #    self.device = ports[0]

        self.g = 9.80665
        self.PI = 3.14159

 #       self.ser = serial.Serial(self.device, 115200, timeout=0.5)
 #       if self.ser:
 #           self.get_logger().info(f"Connected to {self.device}")
 #       else:
 #           self.get_logger().error("Failed to connect to IMU!")
 #           exit(1)

        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)
        self.pub_compas = self.create_publisher(Imu,'/compass', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/magnetometer', 10)
        

        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = self.get_clock().now().to_msg().sec
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.xglob = 0.0
        self.yglob =0.0

        self.imu_connected = 0
        self.timer = self.create_timer(0.5, self.update)


    def update(self):
        if(self.imu_connected < 1 or not self.ser.isOpen()):
            try:
                self.ser = serial.Serial('/dev/wt901', 115200, timeout=1)
        #        self.ser.write(b'\xFF\xAA\x02\x00\x00')
        #        time.sleep(0.05)  # Give it a moment to settle
                self.imu_connected = 1
            except:
                self.get_logger().warning("No USB  Connection to IMU!")
                self.imu_connected = 0
        if(self.imu_connected > 0 and self.ser.isOpen()):
            try:
                # --- Read magnetometer via command FF AA 27 3A 00 ---
                self.ser.write(b'\xFF\xAA\x27\x3A\x00')
                mag_data = self.ser.read(20)
                if len(mag_data) >= 10 and mag_data[0] == 0x55 and mag_data[1] == 0x71:
                    HxL, HxH = mag_data[4], mag_data[5]
                    HyL, HyH = mag_data[6], mag_data[7]
                    HzL, HzH = mag_data[8], mag_data[9]

                    mx = int.from_bytes([HxH, HxL], byteorder="big", signed=True)
                    my = int.from_bytes([HyH, HyL], byteorder="big", signed=True)
                    mz = int.from_bytes([HzH, HzL], byteorder="big", signed=True)
                    print(mx, my, mz)
                else:
                    self.get_logger().warn("Magnetometer data not received correctly!")
                    mx, my, mz = 0, 0, 0  # fallback or leave as None

                heading_rad = math.atan2(my, mx)
                heading_deg = math.degrees(heading_rad)
                if heading_deg < 0:
                    heading_deg += 360

                print(heading_deg)

                # --- Fill IMU message ---
                current_time = self.get_clock().now().to_msg().sec
                dt = 0.02
                self.last_time = current_time

                self.roll = Roll
                self.pitch = Pitch
                self.yaw = Yaw

                imu.angular_velocity = Vector3(x=wx, y=-wy, z=wz)
                imu.linear_acceleration = Vector3(x=ax, y=ay, z=az)

                q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
                imu.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

                imu.header.stamp = self.get_clock().now().to_msg()
                imu.header.frame_id = "imu"
                self.pub_imu.publish(imu)

                # Optionally publish magnetometer separately
                mag_msg = MagneticField()
                mag_msg.magnetic_field = Vector3(x=mx, y=my, z=mz)  # in mG (raw)
                mag_msg.header.stamp = imu.header.stamp
                mag_msg.header.frame_id = "imu"
                self.pub_mag.publish(mag_msg)

            except Exception as e:
                self.get_logger().warning(f"No USB  Connection to IMU! Error: {e}")
                self.imu_connected = 0

    def read_misc(self):
        def decode_signed_16bit(low, high):
            value = (high << 8) | low
            return struct.unpack('<h', struct.pack('<H', value))[0]

        def request_register(ser, reg):
            cmd = bytes([0xFF, 0xAA, 0x27, reg, 0x00])
            ser.write(cmd)

        def read_register_response(ser):
            header = ser.read(2)
            if header != b'\x55\x71':
                return None
            reg_start = ser.read(2)
            data = ser.read(16)
            return reg_start, data

        def parse_magnetometer(data):
            hx = decode_signed_16bit(data[0], data[1])
            hy = decode_signed_16bit(data[2], data[3])
            hz = decode_signed_16bit(data[4], data[5])
            return hx, hy, hz

        request_register(self.ser, 0x3A)  # Magnetometer
        response = read_register_response(self.ser)
        if response is None:
            self.get_logger().warn("Did not receive valid register response!")
            return

        _, mag_data = response
        hx, hy, hz = parse_magnetometer(mag_data)

        # Prepare and publish the ROS2 MagneticField message
        mag_msg = MagneticField()
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = "magnetometer_link"  # or your appropriate frame

        # The units for MagneticField msg are in Tesla (T)
        # If your sensor outputs in different units, convert accordingly.
        # Here assuming raw units need scaling, adjust scale factor if needed.
        scale = 1e-6  # example scaling to Tesla (microTesla to Tesla)
        mag_msg.magnetic_field = Vector3(x=hx * scale, y=hy * scale, z=hz * scale)

        self.mag_pub.publish(mag_msg)
  #      self.get_logger().info(f"Published magnetometer data: x={hx}, y={hy}, z={hz}")

def main(args=None):
    rclpy.init(args=args)
    node = WT901Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()