""" Proportional controller with no reference point, user goal input, and velocity limits """

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class FeedbackController(Node):
    def __init__(self):
        super().__init__('turtlebot3_navcontrol')

        # ROS 2 Publisher & Subscriber
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Initialize goal state (Will be set by user)
        self.goal_x = 0.0  
        self.goal_y = 0.0  
        self.goal_phi = 0.0  

        # Define Control Gains
        self.kp = 0.1  # Proportional gain for linear velocity

        # Robot Max Velocities
        self.max_linear_velocity = 0.22  # Max speed (m/s)
        self.max_angular_velocity = 2.84  # Max angular speed (rad/s)

        # Robot State (Updated from Odometry)
        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0  # Orientation (yaw)

    def set_goal(self):
        """ Asks user for goal input and updates the goal state """
        try:
            self.goal_x = float(input("Enter goal x position: "))
            self.goal_y = float(input("Enter goal y position: "))
            self.goal_phi = float(input("Enter goal orientation (in radians): "))
            self.get_logger().info(f'Goal set to: x={self.goal_x}, y={self.goal_y}, phi={self.goal_phi}')
        except ValueError:
            self.get_logger().error("Invalid input! Please enter numerical values.")

    def odom_callback(self, msg):
        """ Updates the robot's current position and orientation from odometry """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self.phi = math.atan2(siny_cosp, cosy_cosp)

        # Call the custom controller
        self.compute_control()

    def compute_control(self):
        """ Feedback Control Law """

        # Extract current position and orientation
        current_x = self.x
        current_y = self.y
        current_phi = self.phi

        # Extract goal position
        xd = self.goal_x
        yd = self.goal_y

        # Compute desired velocity using feedback control
        xdot_p = self.kp * (xd - current_x)
        ydot_p = self.kp * (yd - current_y)

        # Matrix algebra to compute control inputs
        A = math.cos(current_phi)
        B = math.sin(current_phi)
        C = -math.sin(current_phi)
        D = math.cos(current_phi)

        # Compute control inputs (v and w)
        v = (A * xdot_p) + (B * ydot_p)
        w = (C * xdot_p) + (D * ydot_p)

        # Apply velocity limits
        v = max(min(v, self.max_linear_velocity), -self.max_linear_velocity)  # Clamp linear velocity
        w = max(min(w, self.max_angular_velocity), -self.max_angular_velocity)  # Clamp angular velocity

        # Stop if close to goal
        if abs(self.goal_x - current_x) < 0.05 and abs(self.goal_y - current_y) < 0.05:
            v = 0.0
            w = 0.0

        # Publish control inputs
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_pub.publish(twist)

        # Debugging Output
        self.get_logger().info(f'Goal: ({self.goal_x}, {self.goal_y}), Position: ({self.x:.2f}, {self.y:.2f}), v={v:.2f}, w={w:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = FeedbackController()

    # Ask user for goal input before running
    node.set_goal()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
