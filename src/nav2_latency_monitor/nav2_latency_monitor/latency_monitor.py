import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from collections import deque
import time

class LatencyMonitor(Node):
    def __init__(self):
        super().__init__('latency_monitor')

        self.odom_buffer = deque(maxlen=100)
        self.latency_data = []

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.get_logger().info('Latency Monitor Node Started')

    def odom_callback(self, msg):
        self.odom_buffer.append((self.get_clock().now(), msg))

    def cmd_vel_callback(self, msg):
        cmd_time = self.get_clock().now()

        # Match cmd_vel to most recent odom timestamp
        if self.odom_buffer:
            odom_time, _ = self.odom_buffer[-1]
            latency = (cmd_time - odom_time).nanoseconds / 1e6  # ms
            self.latency_data.append(latency)
            self.get_logger().info(f'Latency: {latency:.2f} ms')

def main(args=None):
    rclpy.init(args=args)
    node = LatencyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nExiting Latency Monitor...")
    finally:
        if node.latency_data:
            avg_latency = sum(node.latency_data) / len(node.latency_data)
            print(f"Average Latency: {avg_latency:.2f} ms over {len(node.latency_data)} samples")
        node.destroy_node()
        rclpy.shutdown()

