import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped

def execute(self, inputs, outputs, gvm):

    drive_cmd = WheelsCmdStamped()
    node = gvm.get_variable("Node", True)
    wheels_cmd_pub = node.create_publisher(WheelsCmdStamped, '/None/wheels_driver_node/wheels_cmd', 10)
    drive_cmd.vel_right = 0.0
    drive_cmd.vel_left = 0.4
    wheels_cmd_pub.publish(drive_cmd)
    self.preemptive_wait(3)
    return "success"