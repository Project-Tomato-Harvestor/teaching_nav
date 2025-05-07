#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
from nav_msgs.msg import Odometry

class OdomCSVRecorder(Node):
    def __init__(self):
        super().__init__('odom_csv_recorder')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.topic_callback,
            10)
        self.subscription

        # Define CSV file path
        self.csv_file = 'odom_data.csv'
        self.first_write = not os.path.exists(self.csv_file)
        
        # Open CSV file in append mode
        self.file = open(self.csv_file, 'a', newline='')
        self.writer = csv.writer(self.file)
        
        # Write header if file is newly created
        if self.first_write:
            self.writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

    def topic_callback(self, msg):
        # Extract odometry data
        timestamp = self.get_clock().now().nanoseconds
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Write data to CSV file
        self.writer.writerow([timestamp, position.x, position.y, position.z, 
                              orientation.x, orientation.y, orientation.z, orientation.w])
        self.file.flush()

    def destroy_node(self):
        # Close the file when shutting down
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    odom_csv_recorder = OdomCSVRecorder()
    try:
        rclpy.spin(odom_csv_recorder)
    except KeyboardInterrupt:
        pass
    finally:
        odom_csv_recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

