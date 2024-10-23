#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import rosbag2_py
import time

class OdomBagReplayer(Node):
    def __init__(self):
        super().__init__('bag_replayer')

        # Set up the reader for the bag file
        self.reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri='odom_bag',  # Change this if your bag file has a different name
            storage_id='sqlite3')
        self.reader.open(storage_options, rosbag2_py._storage.ConverterOptions('', ''))

        # Create a publisher for the /odom topic
        self.publisher = self.create_publisher(Odometry, '/replayed_odom', 10)
        self.get_logger().info("OdomBagReplayer initialized and ready to replay odometry data.")

        # Start replaying the bag
        self.replay_bag()

    def replay_bag(self):
        while True:
            # Read messages from the bag file
            (topic, msg, t) = self.reader.read_next()
            if topic is None:  # If no more messages
                break

            # Publish the message to the /odom topic
            self.publisher.publish(msg)
            self.get_logger().info(f"Replaying message on topic '{topic}'.")

            # Sleep to maintain original timestamp
            time.sleep(t / 1_000_000_000.0)  # Convert nanoseconds to seconds

def main(args=None):
    rclpy.init(args=args)
    odom_bag_replayer = OdomBagReplayer()
    try:
        rclpy.spin(odom_bag_replayer)
    except KeyboardInterrupt:
        odom_bag_replayer.get_logger().info("Replayer stopped by user.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
