#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from nav_msgs.msg import Odometry

import rosbag2_py

class OdomBagRecorder(Node):
    def __init__(self):
        super().__init__('recorder_node')
        self.writer = rosbag2_py.SequentialWriter()

        # Set up bag file storage and format
        storage_options = rosbag2_py._storage.StorageOptions(
            uri='odom_bag',
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        # Define the topic to record (/odom topic)
        topic_info = rosbag2_py._storage.TopicMetadata(
            name='/odom',
            type='nav_msgs/msg/Odometry',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)

        # Subscribe to the /odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.topic_callback,
            10)
        self.subscription

    def topic_callback(self, msg):
        # Write incoming odometry data to the bag file
        self.writer.write(
            '/odom',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)


def main(args=None):
    rclpy.init(args=args)
    odom_bag_recorder = OdomBagRecorder()
    rclpy.spin(odom_bag_recorder)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
