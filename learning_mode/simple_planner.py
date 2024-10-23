#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class SimplePlanner(Node):
    def __init__(self):
        super().__init__('simple_planner')

        # Subscribe to desired pose (from replayed rosbag) and localization data
        self.create_subscription(PoseStamped, '/replayed_odom', self.desired_pose_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_pose = None
        self.desired_pose = None

        # Gain values are directly set here
        self.linear_gain = 1.0  # Set linear gain here
        self.angular_gain = 2.0  # Set angular gain here

    def desired_pose_callback(self, msg):
        self.desired_pose = msg

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.compute_velocity()

    def compute_velocity(self):
        if self.current_pose is None or self.desired_pose is None:
            return

        # Compute errors
        dx = self.desired_pose.pose.position.x - self.current_pose.position.x
        dy = self.desired_pose.pose.position.y - self.current_pose.position.y

        # Compute Euclidean distance and angle difference
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        current_yaw = self.get_yaw_from_pose(self.current_pose)
        angle_diff = angle_to_goal - current_yaw

        # Control law with stopping condition
        if distance < 0.1:  # Threshold for stopping (e.g., 10 cm)
            velocity = 0.0
            omega = 0.0
        else:
            velocity = self.linear_gain * distance
            omega = self.angular_gain * angle_diff

        # Create and publish Twist message
        cmd_msg = Twist()
        cmd_msg.linear.x = velocity
        cmd_msg.angular.z = omega
        self.cmd_vel_publisher.publish(cmd_msg)

    def get_yaw_from_pose(self, pose):
        # Extract yaw from quaternion
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
