import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import time
import math

class PosePlotter(Node):
    def __init__(self):
        super().__init__('performance_plotter')
        
        # Subscribe to Odometry for robot position
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Data storage
        self.time_data = []
        self.x_data = []
        self.y_data = []
        self.linear_speed_data = []
        self.angular_speed_data = []

        # Motion tracking
        self.recording = False
        self.motion_start_time = None

    def odom_callback(self, msg):
        """ Callback for odometry data (robot position updates) """

        # Get linear and angular velocity
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular

        # Compute speed magnitudes
        linear_speed = math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)
        angular_speed = math.sqrt(angular.x**2 + angular.y**2 + angular.z**2)

        current_time = time.time()

        # Thresholds to avoid false triggers from noise
        linear_threshold = 0.01
        angular_threshold = 0.01

        # Start recording if any motion is detected
        if not self.recording:
            if linear_speed > linear_threshold or angular_speed > angular_threshold:
                self.recording = True
                self.motion_start_time = current_time
                self.get_logger().info("Motion detected (linear or angular). Recording started.")
            else:
                return  # Still idle, don't record

        # Record data relative to motion start time
        relative_time = current_time - self.motion_start_time
        self.time_data.append(relative_time)
        self.x_data.append(msg.pose.pose.position.x)
        self.y_data.append(msg.pose.pose.position.y)
        self.linear_speed_data.append(linear_speed)
        self.angular_speed_data.append(angular_speed)

    def plot_results(self):
        """ Plot the final results after termination """
        if not self.time_data:
            self.get_logger().warn("No motion detected. No data to plot.")
            return

        # === Plot X and Y positions over time ===
        fig1, ax1 = plt.subplots()
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Position')
        ax1.set_title('Robot X and Y Position vs Time')
        ax1.plot(self.time_data, self.x_data, label='X Position', color='b')
        ax1.plot(self.time_data, self.y_data, label='Y Position', color='g')
        ax1.legend()
        ax1.grid()

        # === Plot X vs Y trajectory ===
        fig2, ax2 = plt.subplots()
        ax2.set_xlabel('X Position')
        ax2.set_ylabel('Y Position')
        ax2.set_title('Robot Trajectory (X vs Y)')
        ax2.plot(self.x_data, self.y_data, label='Trajectory', color='purple')
        ax2.legend()
        ax2.grid()
        ax2.axis('equal')

        # === Plot linear speed over time ===
        fig3, ax3 = plt.subplots()
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Linear Speed (m/s)')
        ax3.set_title('Linear Speed vs Time')
        ax3.plot(self.time_data, self.linear_speed_data, color='orange')
        ax3.grid()

        # === Plot angular speed over time ===
        fig4, ax4 = plt.subplots()
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Angular Speed (rad/s)')
        ax4.set_title('Angular Speed vs Time')
        ax4.plot(self.time_data, self.angular_speed_data, color='red')
        ax4.grid()

        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = PosePlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_results()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
