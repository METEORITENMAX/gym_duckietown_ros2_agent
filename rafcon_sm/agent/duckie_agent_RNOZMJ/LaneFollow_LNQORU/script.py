import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import String, Bool
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

class LF(Node):

    def __init__(self):
        super().__init__('LaneFollow')
        self.pub_drive_cmd_ = self.create_publisher(WheelsCmdStamped, '/None/wheels_driver_node/wheels_cmd', 10)
        self.sub_dir_ = self.create_subscription(String, '/Drive/Direction', self.sub_dir_cb, 10)
        self.sub_redline_ = self.create_subscription(Bool, '/RedLine/InFront', self.sub_redline_cb, 10)
        self.stop = False
        self.j = 0
    def sub_dir_cb(self, msg):
        drive_cmd = WheelsCmdStamped()
        if msg.data == "left":
                drive_cmd.vel_left = 0.0
                drive_cmd.vel_right = 0.2
        elif msg.data == "right":
                drive_cmd.vel_left = 0.2
                drive_cmd.vel_right = 0.0
        else:
            drive_cmd.vel_left = 0.4
            drive_cmd.vel_right = 0.4
        self.pub_drive_cmd_.publish(drive_cmd)
    def sub_redline_cb(self, msg):
        if msg.data == True:
            drive_cmd = WheelsCmdStamped()
            drive_cmd.vel_left = 0.0
            drive_cmd.vel_right = 0.0
            self.stop = True
            self.pub_drive_cmd_.publish(drive_cmd)


def execute(self, inputs, outputs, gvm):
    
    lf = LF()
    while not lf.stop:
        rclpy.spin_once(lf)
    return "success"
    