import rclpy
import sys
from rclpy.node import Node
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

class LF(Node):

    def __init__(self):
        super().__init__('LaneFollow')
        self.pub_drive_cmd_ = self.create_publisher(WheelsCmdStamped, '/None/wheels_driver_node/wheels_cmd', 10)
        self.sub_img_ = self.create_subscription(CompressedImage, '/None/corrected_image/compressed', self.sub_img_cb, 10 )
        self.red_line_detected = False
        self.j = 0
    def sub_img_cb(self, msg):
        image = cv2.imdecode(np.array(msg.data), 1)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower = np.array([20, 100, 100], dtype="uint8")
        upper = np.array([30, 255, 255], dtype="uint8")
        mask = cv2.inRange(image, lower, upper)

        r_low = np.array([170, 100, 0], dtype="uint8")
        r_up = np.array([180, 255, 255], dtype="uint8")
        mask2 = cv2.inRange(image, r_low, r_up)

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        red_obj_con = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_obj_con = red_obj_con[0] if len(red_obj_con) == 2 else red_obj_con[1]
        drive_cmd = WheelsCmdStamped()
        if len(cnts) != 0:
            x, y, w, h = cv2.boundingRect(cnts[0])
            if x < 100:
                print("left")
                drive_cmd.vel_left = 0.0
                drive_cmd.vel_right = 0.2
            elif x > 200:
                print("right")
                drive_cmd.vel_left = 0.2
                drive_cmd.vel_right = 0.0
            else:
                print("forward")
                drive_cmd.vel_left = 0.4
                drive_cmd.vel_right = 0.4
        if len(red_obj_con) != 0 and self.j > 20:
            x, y, w, h = cv2.boundingRect(red_obj_con[0])
            print("red obj size: " + str(h * w) + " at: " + str(y))
            if h * w > 5000 and y > 340:
                drive_cmd.vel_left = 0.0
                drive_cmd.vel_right = 0.0
                self.pub_drive_cmd_.publish(drive_cmd)
                print("Stop")
                self.red_line_detected = True
        self.j += 1
        self.pub_drive_cmd_.publish(drive_cmd)

def execute(self, inputs, outputs, gvm):
    
    lf = LF()
    while not lf.red_line_detected:
        rclpy.spin_once(lf)
    return "success"
    