#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class BurgerTestNode(Node):
    def __init__(self):
        super().__init__('burger_test')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Burger Test Node has been started")

    def move_robot(self, linear_speed, angular_speed, duration):
        """
        Sends velocity commands to move the robot.
        
        :param linear_speed: Speed in the x-direction (m/s)
        :param angular_speed: Angular speed (rad/s)
        :param duration: Time to maintain this velocity (seconds)
        """
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed

        end_time = time.time() + duration
        while time.time() < end_time:
            self.publisher.publish(msg)
            time.sleep(0.1)  # 10 Hz loop

        # Stop the robot after the motion
        self.stop_robot()

    def stop_robot(self):
        """
        Stops the robot by sending zero velocity.
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("Robot stopped")

    def run_test(self):
        """
        Executes the test sequence: forward, turn, backward.
        """
        # Move forward for approximately 1 meter
        self.get_logger().info("Moving forward")
        self.move_robot(linear_speed=0.2, angular_speed=0.0, duration=5.0)  # Adjust duration based on speed

        # Perform a 360-degree turn
        self.get_logger().info("Performing 360-degree turn")
        self.move_robot(linear_speed=0.0, angular_speed=1.0, duration=6.28)  # 2π radians ~ 360 degrees

        # Move backward for approximately 1 meter
        self.get_logger().info("Moving backward")
        self.move_robot(linear_speed=-0.2, angular_speed=0.0, duration=5.0)

        # End of test
        self.get_logger().info("Test complete")


def main(args=None):
    rclpy.init(args=args)
    node = BurgerTestNode()
    try:
        node.run_test()
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
