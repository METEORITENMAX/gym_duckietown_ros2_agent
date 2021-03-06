import rclpy
from rclpy.node import Node
from std_msgs.msg import String,Bool, Int64

from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
import numpy as np
import os
import cv2
from PIL import Image


class LaneFollow(Node):

    def __init__(self):
        super().__init__('Lane_Follow')
        self.pub_drive_direction_ = self.create_publisher(String, '/Drive/Direction', 10)
        self.red_line_detected_ = self.create_publisher(Bool, '/RedLine/InFront', 10)
        self.mid_lane = self.create_publisher(Int64, '/MidLane', 10)
        #self.pub_drive_cmd_ = self.create_publisher(WheelsCmdStamped, '/None/wheels_driver_node/wheels_cmd', 10)
        self.sub_img_ = self.create_subscription(CompressedImage, '/None/corrected_image/compressed', self.sub_img_cb, 10 )
        timer_period = 1.0  # seconds
        #self.timer = self.create_timer(timer_period, self.sub_img_cb)
        self.i = 0

    def sub_img_cb(self, msg):
        image = cv2.imdecode(np.array(msg.data), 1)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower = np.array([22, 93, 0], dtype="uint8")
        upper = np.array([45, 255, 255], dtype="uint8")
        mask = cv2.inRange(image, lower, upper)

        r_low = np.array([170, 100, 0], dtype="uint8")
        r_up = np.array([180, 255, 255], dtype="uint8")
        mask2 = cv2.inRange(image, r_low, r_up)

        red_obj_con = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        red_obj_con = red_obj_con[0] if len(red_obj_con) == 2 else red_obj_con[1]


        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        drive_dir = String()
        redline = Bool()
        midlane = Int64()
        redline.data = False
        x1 = 0
        if len(cnts) != 0:
            x,y,w,h = cv2.boundingRect(cnts[0])
            x1 = x
            midlane.data = x
            if y > 300:
                if x < 100:
                    drive_dir.data = "left"
                elif x > 200:
                    drive_dir.data = "right"
                else:
                    drive_dir.data = "forward"
        else:
            midlane.data = 0
        self.mid_lane.publish(midlane)
        self.pub_drive_direction_.publish(drive_dir)
        if len(red_obj_con) != 0:
            for red_obj in red_obj_con:
                x, y, w, h = cv2.boundingRect(red_obj)
                if h * w > 3000 and y > 320 and x1 < x:
                    redline.data = True
                    self.red_line_detected_.publish(redline)
            self.i += 1






def main(args=None):
    rclpy.init(args=args)

    lf = LaneFollow()
    lf.get_logger().info('Lane Follow Node Started')
    rclpy.spin(lf)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
