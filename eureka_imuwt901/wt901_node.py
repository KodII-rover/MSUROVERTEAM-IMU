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

        self.timer = self.create_timer(0.02, self.update)

    def update(self):
        if(self.imu_connected < 1 or not self.ser.isOpen()):
            try:
                self.ser = serial.Serial('/dev/wt901', 115200, timeout=1)
                self.imu_connected = 1
            except:
                self.get_logger().warning("No USB  Connection to IMU!")
                self.imu_connected = 0
        if(self.imu_connected > 0 and self.ser.isOpen()):
            try:
                imu = Imu()
                compas = Imu()
                imu.angular_velocity_covariance = self.angular_velocity_covariance
                imu.linear_acceleration_covariance = self.linear_acceleration_covariance
                imu.orientation_covariance = self.orientation_covariance
                data = self.ser.read(2)
                if len(data) < 2 or data[1] != 0x61:
                    self.get_logger().warn("Received incomplete data 2!")
                    raise  

                data = self.ser.read(18)
                if len(data) != 18:
                    self.get_logger().warn("Received incomplete data 18!")
                    raise

                axL, axH, ayL, ayH, azL, azH, wxL, wxH, wyL, wyH, wzL, wzH, RollL, RollH, PitchL, PitchH, YawL, YawH = data

                ax = int.from_bytes([axH, axL], byteorder="big", signed=True) / 32768 * 16 * self.g
                ay = int.from_bytes([ayH, ayL], byteorder="big", signed=True) / 32768 * 16 * self.g
                az = int.from_bytes([azH, azL], byteorder="big", signed=True) / 32768 * 16 * self.g

                wx = int.from_bytes([wxH, wxL], byteorder="big", signed=True) / 32768 * 2000 / 180 * self.PI
                wy = int.from_bytes([wyH, wyL], byteorder="big", signed=True) / 32768 * 2000 / 180 * self.PI
                wz = int.from_bytes([wzH, wzL], byteorder="big", signed=True) / 32768 * 2000 / 180 * self.PI

                Roll = int.from_bytes([RollH, RollL], byteorder="big", signed=True) / 32768 * self.PI
                Pitch = int.from_bytes([PitchH, PitchL], byteorder="big", signed=True) / 32768 * self.PI
                Yaw = int.from_bytes([YawH, YawL], byteorder="big", signed=True) / 32768 * self.PI
                compas.angular_velocity =Vector3(x=Roll, y=Pitch, z=Yaw)
                compas.header.stamp = self.get_clock().now().to_msg()
                compas.header.frame_id = 'compas'
                self.pub_compas.publish(compas)

                current_time = self.get_clock().now().to_msg().sec
                dt = 0.02
                self.last_time = current_time

                self.roll = Roll
                self.pitch = Pitch
                self.yaw = Yaw

                imu.angular_velocity = Vector3(x=wx, y=-wy, z=wz)
                imu.linear_acceleration = Vector3(x=ax,y=ay, z=az )

                q = quaternion_from_euler(self.roll, self.pitch, self.yaw)
                imu.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

                imu.header.stamp = self.get_clock().now().to_msg()
                imu.header.frame_id = "imu"
                
      #          print(imu)

                self.pub_imu.publish(imu)
            except:
                self.get_logger().warning("No USB  Connection to IMU!")
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