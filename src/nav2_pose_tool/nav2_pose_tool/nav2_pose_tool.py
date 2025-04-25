import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import matplotlib.pyplot as plt
import numpy as np
import tf_transformations
import yaml
import cv2
import os
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

def print_description():
    print("""
[Nav2 Pose Tool]
This program acts as a lightweight substitute for RViz when using the Nav2 stack in ROS 2 Humble.

- A valid YAML map file is required to run this node.
- Navigation in completely unknown or unmapped spaces is not supported.
- This tool displays the map with a coordinate grid and allows manual control of the robot.
- To view current robot pose, open a separate terminal and run: ros2 topic echo /amcl_pose.

Usage Instructions:
1. Set an initial pose [Option 1] to localize the robot.
2. Send a goal pose [Option 2] to command navigation.
3. Re-display the map with current information [Option 3].
4. Quit safely at any time with [q].

Please provide the correct path to your map.yaml file when prompted.
    """)

def get_map_path_from_user():
    while True:
        default_path = os.path.expanduser('~/map/map.yaml')
        print(f"Enter path to map.yaml [default: {default_path}]:")
        user_input = input("> ").strip()
        path = user_input if user_input else default_path
        if os.path.exists(path):
            try:
                with open(path, 'r') as file:
                    yaml.safe_load(file)
                return path
            except Exception:
                print("[ERROR] Invalid YAML file. Try again.")
        else:
            print("[ERROR] Map file not found. Try again.")

def aligned_range(start, end, step=0.5):
    aligned_start = np.floor(start / step) * step
    return np.arange(aligned_start, end, step)

class Nav2PoseTool(Node):
    def __init__(self, map_yaml_path):
        super().__init__('nav2_pose_tool')

        self.map_yaml_path = map_yaml_path
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = [0.0, 0.0]
        self.robot_pose = None
        self.last_initial_pose = None

        self.load_map()

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            qos_profile
        )

        self.display_map_with_grid()
        self.user_interface()

    def load_map(self):
        while True:
            try:
                with open(self.map_yaml_path, 'r') as file:
                    map_yaml = yaml.safe_load(file)
                    map_image_path = os.path.join(os.path.dirname(self.map_yaml_path), map_yaml['image'])
                    self.map_resolution = map_yaml['resolution']
                    self.map_origin = map_yaml['origin'][:2]

                map_image = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
                if map_image is None:
                    raise FileNotFoundError(f"Failed to load image at {map_image_path}")

                self.map_data = cv2.flip(map_image, 0)
                break
            except Exception as e:
                print(f"[ERROR] Failed to load map: {e}")
                self.map_yaml_path = get_map_path_from_user()

    def amcl_pose_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def display_map_with_grid(self):
        if self.map_data is None:
            print("[ERROR] Map data not loaded. Cannot display.")
            return

        height, width = self.map_data.shape
        extent = [
            self.map_origin[0],
            self.map_origin[0] + width * self.map_resolution,
            self.map_origin[1],
            self.map_origin[1] + height * self.map_resolution
        ]

        plt.figure(figsize=(10, 10))
        plt.imshow(self.map_data, cmap='gray', origin='lower', extent=extent)
        plt.title("Map with Coordinate Grid (in meters)")
        plt.xlabel("X [meters]")
        plt.ylabel("Y [meters]")

        x_ticks = aligned_range(extent[0], extent[1])
        y_ticks = aligned_range(extent[2], extent[3])
        for x in x_ticks:
            plt.axvline(x, color='skyblue', linestyle='--', linewidth=0.8)
        for y in y_ticks:
            plt.axhline(y, color='skyblue', linestyle='--', linewidth=0.8)

        if self.last_initial_pose:
            x = self.last_initial_pose.position.x
            y = self.last_initial_pose.position.y
            plt.plot(x, y, 'ro', markersize=8)
            orientation_q = self.last_initial_pose.orientation
            _, _, yaw = tf_transformations.euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            ])
            arrow_length = 0.5
            plt.arrow(x, y, arrow_length * np.cos(yaw), arrow_length * np.sin(yaw),
                      head_width=0.2, head_length=0.2, fc='red', ec='red')
            plt.text(x + 0.2, y + 0.2, "Initial Pose", color='red', fontsize=9)

        plt.grid(False)
        plt.show()

    def user_interface(self):
        while rclpy.ok():
            print("\n[Nav2 Pose Tool]")
            print("Enter a command:\n  [1] Set initial pose\n  [2] Send goal pose\n  [3] Show map again\n  [q] Quit")
            choice = input("> ")

            if choice == '1':
                self.set_initial_pose()
            elif choice == '2':
                self.send_goal_pose()
            elif choice == '3':
                self.display_map_with_grid()
            elif choice.lower() == 'q':
                print("Exiting...")
                rclpy.shutdown()
                break
            else:
                print("[ERROR] Invalid choice.")

    def set_initial_pose(self):
        try:
            pose_input = input("Enter initial pose (x y yaw_degrees): ").strip().split()
            if len(pose_input) != 3:
                raise ValueError("Exactly 3 values required (x y yaw_degrees).")
            x, y, yaw_deg = map(float, pose_input)
            q = tf_transformations.quaternion_from_euler(0, 0, np.radians(yaw_deg))
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = 'map'
            pose.pose.pose.position.x = x
            pose.pose.pose.position.y = y
            pose.pose.pose.orientation.x = q[0]
            pose.pose.pose.orientation.y = q[1]
            pose.pose.pose.orientation.z = q[2]
            pose.pose.pose.orientation.w = q[3]
            pose.pose.covariance[0] = 0.25
            pose.pose.covariance[7] = 0.25
            pose.pose.covariance[35] = 0.0685
            self.last_initial_pose = pose.pose.pose
            self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10).publish(pose)
            print("[INFO] Initial pose set.")
        except Exception as e:
            print(f"[ERROR] Failed to set pose: {e}")

    def send_goal_pose(self):
        try:
            pose_input = input("Enter goal pose (x y yaw_degrees): ").strip().split()
            if len(pose_input) != 3:
                raise ValueError("Exactly 3 values required (x y yaw_degrees).")
            x, y, yaw_deg = map(float, pose_input)
            q = tf_transformations.quaternion_from_euler(0, 0, np.radians(yaw_deg))
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]
            self.navigator.goToPose(goal)
            print("[INFO] Goal sent.")
        except Exception as e:
            print(f"[ERROR] Failed to send goal: {e}")

def main():
    rclpy.init()
    print_description()  # <<< Print description BEFORE asking for map
    map_yaml_path = get_map_path_from_user()
    Nav2PoseTool(map_yaml_path)
    rclpy.spin()

if __name__ == '__main__':
    main()
