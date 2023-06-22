#!/usr/bin/env python3

# lib example
from python_lib.lib_example import hello

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

def main():
    hello()
    rclpy.init()
    test_node = Node("node_py_name")

    str_publisher = test_node.create_publisher(String, "/topicdd", 10)

    msg = String()
    msg.data = "hello"
    for i in range(10):
        str_publisher.publish(msg)
        hello()
        time.sleep(1)

    return True


if __name__ == "__main__":
    main()


