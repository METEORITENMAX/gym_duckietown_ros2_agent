import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import String

def execute(self, inputs, outputs, gvm):

    node = gvm.get_variable("Node", True)
    test_pub = node.create_publisher(String, 'topic', 10)
    msg = String()
    msg.data = 'Hello World' 
    test_pub.publish(msg)
    
    return "success"
    