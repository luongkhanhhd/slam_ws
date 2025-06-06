#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import serial
import math

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        try:
            self.serial = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
            self.get_logger().info("Connected to serial port /dev/ttyUSB1")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            raise
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            line = self.serial.readline().decode('utf-8').strip()
            if line.startswith('ODOM:'):
                data = line[5:].split(',')
                if len(data) == 6:
                    x, y, theta, vx, vy, wz = map(float, data)
                    
                    # Tạo message Odometry
                    odom = Odometry()
                    odom.header.stamp = self.get_clock().now().to_msg()
                    odom.header.frame_id = 'odom'
                    odom.child_frame_id = 'base_link'
                    
                    # Thiết lập vị trí
                    odom.pose.pose.position.x = x
                    odom.pose.pose.position.y = y
                    odom.pose.pose.position.z = 0.0
                    
                    # Chuyển đổi góc Euler sang Quaternion
                    q = self.quaternion_from_euler(0, 0, theta)
                    odom.pose.pose.orientation.x = q[0]
                    odom.pose.pose.orientation.y = q[1]
                    odom.pose.pose.orientation.z = q[2]
                    odom.pose.pose.orientation.w = q[3]
                    
                    # Thiết lập vận tốc
                    odom.twist.twist.linear.x = vx
                    odom.twist.twist.linear.y = vy
                    odom.twist.twist.angular.z = wz
                    
                    self.odom_pub.publish(odom)

                    # Tạo TransformStamped để broadcast TF
                    t = TransformStamped()
                    t.header.stamp = odom.header.stamp
                    t.header.frame_id = 'odom'
                    t.child_frame_id = 'base_link'
                    t.transform.translation.x = x
                    t.transform.translation.y = y
                    t.transform.translation.z = 0.0
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]
                    
                    self.tf_broadcaster.sendTransform(t)
            elif line == 'OK':
                self.get_logger().info("Received OK from controller")
            else:
                self.get_logger().warn(f"Invalid data: {line}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
        except ValueError as e:
            self.get_logger().error(f"Data parsing error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    try:
        node = OdomPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if hasattr(node, 'serial'):
            node.serial.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
