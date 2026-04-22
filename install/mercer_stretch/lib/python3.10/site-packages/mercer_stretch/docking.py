#! /usr/bin/env python
import mercer_stretch.mercer_aruco_to_nav_pose as an
import rclpy
import time

from geometry_msgs.msg import Twist
from rclpy.duration import Duration

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('aruco_docking_node')
    
    aruco = an.ArucoNavigationNode()

    success = aruco.go_to_pose("dock")

    cmd_vel_pub = node.create_publisher(Twist, '/stretch/cmd_vel', 10)

    if success:
        node.get_logger().info("Reached the docking position. Start getting in")
        twist = Twist()
        twist.linear.x = -0.05  # Move backward at 5 cm/s

        # Calculate the duration for moving 1 meters at 5 cm/s (10 seconds)
        duration = 1 / 0.05
        start_time = node.get_clock().now()  # Get the current time in ROS2

        # Move the robot backward for the calculated duration
        while node.get_clock().now() < (start_time + Duration(seconds=duration)):
            cmd_vel_pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.25)

        node.get_logger().info("Docking process completed.")
    
    else:
        node.get_logger.info("Failed to reached the docking position")

    node.destroy_node()
    aruco.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()