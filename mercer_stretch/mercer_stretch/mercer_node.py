import time
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Transform

from stretch_core.keyboard import KBHit


class MercerNode(Node):
    
    def __init__(self):
        super().__init__("mercer_node")

        self.is_keyboard_running = True
        self.keyboard_status_sub = self.create_subscription(String, "/keyboard_listener/status", self.keyboard_listener_callback, 1)
        self.keyboard_status_sub

        self.keyboard_buffer_timer = self.create_timer(10, self.keyboard_buffer_timer_callback)

        self.keyboard_status_timer = self.create_timer(2, self.keyboard_status_timer_callback)
        self.keyboard_status_timer.cancel()


    def keyboard_buffer_timer_callback(self):
        self.get_logger().info("Keyboard Teleop was inactive for 10 seconds... Starting ArUco Alignment")
        self.keyboard_buffer_timer.cancel()
        self.is_keyboard_running = False

        self.align_to_aruco()


    def keyboard_status_timer_callback(self):
        if 'keyboard_listener' not in self.get_node_names():
            self.get_logger().info("Keyboard Listener Node is no longer reachable... Starting ArUco Alignment")
            self.is_keyboard_running = False
            self.keyboard_status_timer.cancel()

            self.align_to_aruco()


    def keyboard_listener_callback(self, msg):
        if msg.data == 'active':
            self.get_logger().info("Keyboard Teleop is active! Restarting keyboard_status_timer")
            self.is_keyboard_running = True

            if not self.keyboard_buffer_timer.is_canceled(): self.keyboard_buffer_timer.cancel()
            
            if self.keyboard_status_timer.is_canceled(): self.keyboard_status_timer.reset()

        elif msg.data == 'inactive':
            self.get_logger().info("Keyboard Teleop is now inactive... Starting ArUco Alignment")
            self.is_keyboard_running = False

            if not self.keyboard_status_timer.is_canceled(): self.keyboard_status_timer.cancel()
            if not self.keyboard_buffer_timer.is_canceled(): self.keyboard_buffer_timer.cancel()

            self.align_to_aruco()
   

    def align_to_aruco(self):
        self.get_logger().info("Aligning to ArUco now! Test Finished")
        # Make this an Action with a Server and Client setup.


def main():
    try:
        rclpy.init()
        node = MercerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().error("Keyboard interrupt recieved - shutting down mercer node")
    finally:
        node.destroy_node()

    rclpy.shutdown()
    sys.exit()


if __name__ == "__main__":
    main()