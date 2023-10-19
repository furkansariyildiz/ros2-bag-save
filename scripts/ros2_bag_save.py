#!/usr/bin/env python3

import os 
import rclpy
import rosbag2_py

from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import serialize_message
from std_msgs.msg import String


class RosbagRecorder(Node):
    def __init__(self):
        super().__init__("ros2_bag_recorder_node")

        self._rosbag_package_direction = os.path.join(get_package_share_directory('ros2_bag_save'), 'bags')

        self._rosbag_writer = rosbag2_py.SequentialWriter()

        self._topics_name = self.declare_parameter('topics_info.topics_name', [""])
        self._message_type = self.declare_parameter('topics_info.message_type', [""])
        self._data_type = self.declare_parameter('topics_info.data_type', [""])
        self._queue_size = self.declare_parameter('topics_info.queue_size', [0])
        self._variable_name = self.declare_parameter('topics_info.variable_name', [""])
        self._rosbag_file_name = self.declare_parameter('bag_file_direction', "/home/$USER/bags/")


        self._topics_name = self.get_parameter('topics_info.topics_name').get_parameter_value().string_array_value
        self._message_type = self.get_parameter('topics_info.message_type').get_parameter_value().string_array_value
        self._data_type = self.get_parameter('topics_info.data_type').get_parameter_value().string_array_value
        self._queue_size = self.get_parameter('topics_info.queue_size').get_parameter_value().integer_array_value
        self._variable_name = self.get_parameter('topics_info.variable_name').get_parameter_value().string_array_value
        self._rosbag_file_name = self.get_parameter('bag_file_direction').get_parameter_value().string_value


        # Debugging Parameters
        self.get_logger().info("Topics Name: " + str(self._topics_name))
        self.get_logger().info("Message Types: " + str(self._message_type))
        self.get_logger().info("Data Types: " + str(self._message_type))
        self.get_logger().info("Queue Size: " + str(self._queue_size))
        self.get_logger().info("Variable Name: " + str(self._variable_name))
        self.get_logger().info("Rosbag File Name Direction: " + str(self._rosbag_file_name))

        storage_options = rosbag2_py.StorageOptions(
            uri=self._rosbag_file_name + "20/",
            storage_id='sqlite3'
        )

        converter_options = rosbag2_py.ConverterOptions('', '')
        self._rosbag_writer.open(storage_options, converter_options)

        topic_info = rosbag2_py.TopicMetadata(
            name='chatter',
            type='std_msgs/msg/String',
            serialization_format='cdr'
        )

        self._rosbag_writer.create_topic(topic_info)

        self._subscription = self.create_subscription(String, 'chatter', self.chatterCallback, 10)


        

    def chatterCallback(self, message):
        self._rosbag_writer.write('chatter', serialize_message(message), self.get_clock().now().nanoseconds)




def main(args=None):
    rclpy.init(args=args)

    ros2_bag_recorder = RosbagRecorder()

    rclpy.spin(ros2_bag_recorder)

    ros2_bag_recorder.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

