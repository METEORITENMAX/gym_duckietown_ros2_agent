import rclpy
import sys
from rclpy.node import Node
import traceback

def execute(self, inputs, outputs, gvm):

        try:
            if not gvm.variable_exist("Node") or not gvm.get_variable("Node"):
                gvm.set_variable("ros_node_initialized", True)
                rclpy.init(args=None)
                node = rclpy.create_node('LaneFollow')
                gvm.set_variable("Node", node, per_reference=True)
                self.logger.info("Creating node: LF")                           
        except Exception as e:
            self.logger.error("Unexpected error:" + str(e) + str(traceback.format_exc()))
                  
        return 0