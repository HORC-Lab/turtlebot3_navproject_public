import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.time import Time
import csv

class DeltaTLogger(Node):
    def __init__(self):
        super().__init__('delta_t_logger')
        self.sub = self.create_subscription(TwistStamped, '/cmd_vel_stamped', self.callback, 10)
        self.last_stamp = None
        self.file = open('delta_t_log.csv', 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['msg_num', 'delta_t_ms'])
        self.count = 0

    def callback(self, msg):
        now = Time.from_msg(msg.header.stamp)
        if self.last_stamp is not None:
            delta_ms = (now.nanoseconds - self.last_stamp.nanoseconds) / 1e6
            self.writer.writerow([self.count, delta_ms])
            self.get_logger().info(f'Delta T: {delta_ms:.2f} ms')
        self.last_stamp = now
        self.count += 1

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = DeltaTLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
