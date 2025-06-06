#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class VelToSerial(Node):
    def __init__(self):
        super().__init__('vel_to_serial')
        self.serial = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x * 100.0  # Chuyển m/s sang scale -100 to 100
        vy = msg.linear.y * 100.0
        wz = msg.angular.z * 100.0
        cmd = f"{vx:.3f},{vy:.3f},{wz:.3f}\n"
        try:
            self.serial.write(cmd.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Error sending serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VelToSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
