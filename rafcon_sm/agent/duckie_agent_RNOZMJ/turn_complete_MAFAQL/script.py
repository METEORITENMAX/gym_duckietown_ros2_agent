from std_msgs.msg import String,Bool, Int64
import rclpy
midlane = True

def sub_midline_cb(msg):
    if msg.data > 200 or msg.data == 0:
        midlane  = False
def execute(self, inputs, outputs, gvm):
    node = gvm.get_variable("Node", True)

    self.sub_redline_ = node.create_subscription(Int64, '/MidLane', sub_midline_cb, 10)
    rclpy.spin_once(node)
    if midlane:
        return "success"
    else:
        return "aborted"