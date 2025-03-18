""" Nonlinear Proportional controller with feedforward control """

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class FeedbackController(Node):
    def __init__(self):
        super().__init__('feedforward_proportional')

        # ROS 2 Publisher & Subscriber
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Initialize goal state (Will be set by user)
        self.goal_x = 0.0  
        self.goal_y = 0.0  
        self.goal_phi = 0.0

        # Gains will be set manually by the user for tuning
        # self.kp = 1.0  # Proportional gain (for nominal controls)
        # self.k1 = 0.3  # Forward correction gain
        # self.k2 = 0.1  # Lateral error gain
        # self.k3 = 0.5  # Orientation error gain
        # self.kpo = 0.5  # Proportional gain for post-nav orientation correction

        # Robot Max Velocities
        self.max_linear_velocity = 0.22  # Max speed (m/s)
        self.max_angular_velocity = 2.84  # Max angular speed (rad/s)

        # Robot State (Updated from Odometry)
        self.x = 0.0
        self.y = 0.0
        self.phi = 0.0  # Orientation (yaw)

        # Manually set x offset
        self.x_offset = 0.0

        # State flag for orientation adjustment
        self.reached_position = False

    def set_offset(self):
        """Asks user for a manual x offset input."""
        try:
            self.x_offset = float(input("Enter x offset: "))
            self.get_logger().info(f'Using x offset: {self.x_offset}')
        except ValueError:
            self.get_logger().error("Invalid input! Using default x offset: 0.0")
            self.x_offset = 0.0

    def set_gains(self):
        """Asks user for control gains input and updates the gains for tuning."""
        try:
            self.kp = float(input("Enter proportional gain kp: "))
            self.k1 = float(input("Enter forward correction gain k1: "))
            self.k2 = float(input("Enter lateral error gain k2: "))
            self.k3 = float(input("Enter orientation error gain k3: "))
            self.kpo = float(input("Enter post-nav orientation correction gain kpo: "))
            self.get_logger().info(f'Gains set to: kp={self.kp}, k1={self.k1}, k2={self.k2}, k3={self.k3}, kpo={self.kpo}')
        except ValueError:
            self.get_logger().error("Invalid input! Please enter numerical values for gains.")

    def set_goal(self):
        """Asks user for goal input and updates the goal state."""
        try:
            self.goal_x = float(input("Enter goal x position: "))
            self.goal_y = float(input("Enter goal y position: "))
            self.goal_phi = float(input("Enter goal orientation (in degrees): "))
            self.goal_phi = math.radians(self.goal_phi)  # Convert to radians
            self.get_logger().info(f'Goal set to: x={self.goal_x}, y={self.goal_y}, phi={math.degrees(self.goal_phi)}°')
        except ValueError:
            self.get_logger().error("Invalid input! Please enter numerical values.")

    def odom_callback(self, msg):
        """Updates the robot's current position and orientation from odometry."""
        # Update the robot's current position and orientation using the manually set offset
        self.x = msg.pose.pose.position.x - self.x_offset
        self.y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self.phi = math.atan2(siny_cosp, cosy_cosp)

        # Call the custom controller
        self.compute_control()

    def compute_control(self):
        """Feedback Control Law."""
        current_x = self.x
        current_y = self.y
        current_phi = self.phi

        # Compute the desired heading to the goal
        xd = self.goal_x
        yd = self.goal_y
        phid = math.atan2(yd - current_y, xd - current_x)

        # Extract gains
        kp = self.kp
        k1 = self.k1
        k2 = self.k2
        k3 = self.k3

        """ Compute nominal controls """
        xdot_p = kp * (xd - current_x)
        ydot_p = kp * (yd - current_y)

        # Matrix algebra to compute nominal controls
        A = math.cos(current_phi)
        B = math.sin(current_phi)
        C = -math.sin(current_phi)
        D = math.cos(current_phi)

        vd = (A * xdot_p) + (B * ydot_p)
        wd = (C * xdot_p) + (D * ydot_p)

        # Apply velocity limits
        vd = max(min(vd, self.max_linear_velocity), 0.0)
        wd = max(min(wd, self.max_angular_velocity), -self.max_angular_velocity)

        """ Compute control inputs """
        phi_e = math.atan2(math.sin(current_phi - phid), math.cos(current_phi - phid))
        x_e = math.cos(phid) * (current_x - xd) + math.sin(phid) * (current_y - yd)
        y_e = -math.sin(phid) * (current_x - xd) + math.cos(phid) * (current_y - yd)

        cos_phi_e = math.cos(phi_e)
        if abs(cos_phi_e) < 1e-6:
            self.get_logger().warn("cos(phi_e) is near zero, adjusting control to avoid instability.")
            v = 0.0
        else:
            v = (vd - k1 * abs(vd) * (x_e + y_e * math.tan(phi_e))) / cos_phi_e

        w = wd - (k2 * vd * y_e + k3 * abs(vd) * math.tan(phi_e)) * cos_phi_e ** 2

        v = max(min(v, self.max_linear_velocity), 0.0)  
        w = max(min(w, self.max_angular_velocity), -self.max_angular_velocity)

        if abs(xd - current_x) < 0.05 and abs(yd - current_y) < 0.05:
            v = 0.0
            self.reached_position = True

        if self.reached_position:
            w = self.adjust_orientation()
        
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_pub.publish(twist)

        self.get_logger().info(f'Goal: ({self.goal_x}, {self.goal_y}, {math.degrees(self.goal_phi)}°) '
                               f'| Position: ({self.x:.2f}, {self.y:.2f}, {math.degrees(self.phi):.2f}°) '
                               f'| v={v:.2f}, w={w:.2f}')

    def adjust_orientation(self):
        """Adjusts the robot's orientation to the desired final angle."""
        phi_error = math.atan2(math.sin(self.goal_phi - self.phi), math.cos(self.goal_phi - self.phi))
        w = self.kpo * phi_error
        w = max(min(w, self.max_angular_velocity), -self.max_angular_velocity)

        if abs(phi_error) < 0.001:
            w = 0.0
            self.get_logger().info("Final orientation reached. Stopping rotation.")

        return w

def main(args=None):
    rclpy.init(args=args)
    node = FeedbackController()

    # Ask user for manual x offset input before starting the node
    node.set_offset()

    # Ask user for gains input for tuning purposes
    node.set_gains()
    # Ask user for goal input
    node.set_goal()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
