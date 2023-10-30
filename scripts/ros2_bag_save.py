#!/usr/bin/env python3

import os 
import rclpy
import rosbag2_py

from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from datetime import datetime

class RosbagRecorder(Node):
    def __init__(self):
        super().__init__("ros2_bag_recorder_node")

        self._rosbag_package_direction = os.path.join(get_package_share_directory('ros2_bag_save'), 'bags')

        self._rosbag_writer = rosbag2_py.SequentialWriter()

        # Declaering Parameters
        self._topic_names = self.declare_parameter('topics_info.topic_names', [""])
        self._message_types = self.declare_parameter('topics_info.message_types', [""])
        self._data_types = self.declare_parameter('topics_info.data_types', [""])
        self._interface_types = self.declare_parameter('topics_info.interface_types', [""])
        self._callback_functions = self.declare_parameter('topics_info.callback_function', [""])
        self._queue_sizes = self.declare_parameter('topics_info.queue_sizes', [0])
        self._variable_names = self.declare_parameter('topics_info.variable_names', [""])
        self._rosbag_file_name = self.declare_parameter('bag_file_direction', "/home/$USER/bags/")

        # Getting Parameters from Config YAML file
        self._topic_names = self.get_parameter('topics_info.topic_names').get_parameter_value().string_array_value
        self._message_types = self.get_parameter('topics_info.message_types').get_parameter_value().string_array_value
        self._data_types = self.get_parameter('topics_info.data_types').get_parameter_value().string_array_value
        self._interface_types = self.get_parameter('topics_info.interface_types').get_parameter_value().string_array_value
        self._callback_functions = self.get_parameter('topics_info.callback_function').get_parameter_value().string_array_value
        self._queue_sizes = self.get_parameter('topics_info.queue_sizes').get_parameter_value().integer_array_value
        self._variable_names = self.get_parameter('topics_info.variable_names').get_parameter_value().string_array_value
        self._rosbag_file_name = self.get_parameter('bag_file_direction').get_parameter_value().string_value

        self._function_list = {}        

        # Debugging Parameters
        self.debug("Topics Name: " + str(self._topic_names))
        self.debug("Message Types: " + str(self._message_types))
        self.debug("Data Types: " + str(self._data_types))
        self.debug("Callback Functions: " + str(self._callback_functions))
        self.debug("Queue Size: " + str(self._queue_sizes))
        self.debug("Variable Names: " + str(self._variable_names))
        self.debug("Rosbag File Name Direction: " + str(self._rosbag_file_name))

        # Configurating storage options
        self._date_time = str(datetime.now()).replace(" ", "-").replace(":", "-")
        storage_options = rosbag2_py.StorageOptions(uri=self._rosbag_file_name + str(self._date_time) + "/", storage_id='sqlite3')

        # Configurating converter options
        converter_options = rosbag2_py.ConverterOptions('', '')
        self._rosbag_writer.open(storage_options, converter_options)

        # Calling all functions one by one
        self.importLibraries()
        self.defineMessages()
        self.defineClassVariables()
        self.defineCallbackFunctions()
        self.defineSubscribers()



    def importLibraries(self):
        """
        @brief Importing ros2 message (interface) libraries via using parameters which are getted from config.yaml file.
        @param self
        @return None
        """
        for message_type, data_type in zip(self._message_types, self._data_types):
            exec(("from " + message_type + " import " + data_type), globals())


    
    def defineMessages(self):
        """
        @brief Defining messages as class object 
        @param self
        @return None
        """
        for variable_name, data_type in zip(self._variable_names, self._data_types):
            exec("self." + variable_name + "=" + data_type + "()")



    def defineClassVariables(self):
        """
        @brief Defining class variables to get variables on exec() function.
        @param self
        @return None
        """
        for variable_name, data_type in zip(self._variable_names, self._data_types):
            exec("self." + variable_name + "=" + data_type + "()")
    


    def defineCallbackFunctions(self):
        """
        @brief Defining Callback functions to use on ROS2 Subscribers.
        @param self
        @return None
        @source https://stackoverflow.com/questions/51064959/how-to-do-exec-definition-inside-class-python
        """
        for topic_name, variable_name, callback_function_name in zip(self._topic_names, self._variable_names, self._callback_functions):
            exec('def ' + callback_function_name + '(self, message): self.' + variable_name + "= message; self.debug(message); self._rosbag_writer.write('" + topic_name + "', serialize_message(message), "  +  "self.get_clock().now().nanoseconds" + ")", globals(), self._function_list)
        

        for function in self._function_list:
            if not hasattr(self.__class__, function):
                setattr(self.__class__, function, self._function_list[function])



    def defineSubscribers(self):
        """
        @brief Defining subscribers via using topic_name, message_type, data_type, interface_type, callback_function_name and queue_size.
        @param self
        @return None
        """
        for topic_name, message_type, data_type, interface_type, callback_function_name, queue_size in zip(self._topic_names, self._message_types, self._data_types, self._interface_types, self._callback_functions, self._queue_sizes):
            exec("self.create_subscription(" + data_type + ", '" + topic_name + "', " + "self. " + callback_function_name + ", " + str(queue_size) + ")")
            topic_info = rosbag2_py.TopicMetadata(name=str(topic_name), type=str(interface_type), serialization_format='cdr')
            self._rosbag_writer.create_topic(topic_info)



    def debug(self, message):
        """
        @brief Debugging messages
        @param message (string, int etc.) Debugged message
        @return None
        """
        self.get_logger().info("Message: " + str(message))



def main(args=None):
    rclpy.init(args=args)

    ros2_bag_recorder = RosbagRecorder()

    rclpy.spin(ros2_bag_recorder)

    ros2_bag_recorder.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

