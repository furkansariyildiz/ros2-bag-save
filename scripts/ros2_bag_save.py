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

        # self._topics_info = self.declare_parameter('topics_info', [])
        self._rosbag_file_name = self.declare_parameter('bag_file_direction', "/home/$USER/bags/")


        # self._topics_info = self.get_parameter('topics_info').get_parameter_value()
        self._rosbag_file_name = self.get_parameter('bag_file_direction').get_parameter_value().string_value

        print("Rosbag file name: " + str(self._rosbag_file_name))

        storage_options = rosbag2_py.StorageOptions(
            uri=self._rosbag_file_name + "14/",
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

