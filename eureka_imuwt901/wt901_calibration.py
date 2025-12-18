import rclpy
from rclpy.node import Node
import serial
import time

g = 9.80665
PI = 3.14159
timeout = 0.2

DEBUG = True

def send(ser, command: str, delay=timeout):
    ser.write(bytes.fromhex(command))  
    time.sleep(delay)  

class WT901Node(Node):
    def __init__(self):
        super().__init__('wt901_calibration_node')

        self.declare_parameter('wt901/port', '/dev/wt901')

        self.device = self.get_parameter('wt901/port').get_parameter_value().string_value

        try:
            self.ser = serial.Serial(self.device, 115200, timeout=None)
            self.get_logger().info(f"Successfully connected to {self.device}. Waiting for {timeout} sec")
        except Exception as e:
            self.get_logger().error(f"Can not connect to IMU: {e}")
            exit(1)

        time.sleep(timeout)

        self.get_logger().info(f"Accelerometer Calibration. Waiting for 5 sec...")
  #      send(self.ser, 'FF AA 01 01 00', 5) 
  #      send(self.ser, 'FF AA 01 00 00')  
  #      send(self.ser, 'FF AA 00 00 00')  
        self.get_logger().info(f"Calibration finished")

        send(self.ser, 'FF AA 03 07 00') 
        self.get_logger().info(f"Hardware rate has been set up for 50 Hz")

        self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = WT901Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.ser.close()
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()