import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
import numpy as np
import os
import cv2
from PIL import Image


class LaneFollowCalc(Node):

    def __init__(self):
        super().__init__('Lane_Follow')
        self.pub_drive_cmd_ = self.create_publisher(WheelsCmdStamped, '/None/wheels_driver_node/wheels_cmd', 10)
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

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        drive_cmd = WheelsCmdStamped()
        if len(cnts) != 0:
            x,y,w,h = cv2.boundingRect(cnts[0])
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
        self.i += 1
        self.pub_drive_cmd_.publish(drive_cmd)

        self.get_logger().info('I heard: "%s"' )

    def timer_callback(self):
        msg = WheelsCmdStamped()
        h = 'Hello World: %d' % self.i
        msg.vel_left = 0.0
        msg.vel_right = 1.0
        self.pub_drive_cmd_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % h)





def main(args=None):
    rclpy.init(args=args)

    lf_calc = LaneFollowCalc()

    rclpy.spin(lf_calc)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lf_calc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
