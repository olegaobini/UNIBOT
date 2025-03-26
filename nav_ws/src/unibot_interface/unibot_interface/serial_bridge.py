import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Adjust this to match your MCU's port and baud rate
        self.serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        self.get_logger().info('Serial bridge initialized.')

    def cmd_vel_callback(self, msg):
        # Convert m/s to mm/s and rad/s to deg/s
        forward_speed = int(msg.linear.x * 1000)
        yaw_rate = int(msg.angular.z * 180 / 3.14159)

        # Clamp for safety
        forward_speed = max(min(forward_speed, 32767), -32768)
        yaw_rate = max(min(yaw_rate, 32767), -32768)

        # Pack as signed 16-bit integers
        packet = struct.pack('<hh', forward_speed, yaw_rate)

        try:
            self.serial.write(packet)
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

