import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import time

class PosePlotter(Node):
    def __init__(self):
        super().__init__('pose_plotter')
        
        # Subscribe to Odometry for robot position
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Data storage
        self.time_data = []
        self.x_data = []
        self.y_data = []
        self.start_time = time.time()

    def odom_callback(self, msg):
        """ Callback for odometry data (robot position updates) """
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)

    def plot_results(self):
        """ Plot the final results after termination """
        fig, ax = plt.subplots()
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position')
        ax.set_title('Robot X and Y Position vs Time')

        # Plot robot's X and Y positions over time
        ax.plot(self.time_data, self.x_data, label='X Position', color='b')
        ax.plot(self.time_data, self.y_data, label='Y Position', color='g')

        ax.legend()
        plt.grid()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = PosePlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_results()  # Plot data after termination
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
