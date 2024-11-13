#! /usr/bin/env python

#import hello_helpers.hello_misc as hm

import rclpy
import logging
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import time
from math import pi, sqrt, atan2
import json 
import os 

from std_srvs.srv import Trigger

from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from mercer_interfaces.msg import SimplifiedMarkerArray, SimplifiedMarker

class ArucoNavigationNode(Node):
    def __init__(self):
        
        super().__init__('mercer_aruco_to_nav_pose')
        node_name = self.get_name()
        self.get_logger().info("{0} started".format(node_name))

        self.joint_state = None
        self.file_path = '/home/hello-robot/mercer_ws/src/mercer_stretch'
        try:
            saved_file = open(self.file_path + "/config/saved_poses.json")
            self.pose_dict = json.load(saved_file)
            saved_file.close()
        except:
            self.pose_dict = {}

        self.joint_states_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.joint_states_sub
        # self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.client.wait_for_server()

        self.visible_markers = set()
        self.marker_listener = self.create_subscription(SimplifiedMarkerArray, '/aruco/visible_markers', self.marker_callback, 1)
        self.marker_listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.trigger_listener = self.create_subscription(String, '/aruco/trigger_save', self.save_pose_callback, 1)
        self.trigger_listener        

    def pose_to_list(self, pose):
        '''
        Reformats a PoseStamped message into a list that can be saved as a dictionary entry
        '''
        
        return [pose.header.frame_id, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]

    def list_to_pose(self, list):
        '''
        Reformats a pose dictionary entry into the original PoseStamped message
        '''
        
        pose = PoseStamped()
        pose.header.frame_id = list[0]
        pose.pose.position.x = list[1]
        pose.pose.position.y = list[2]
        pose.pose.position.z = list[3]
        pose.pose.orientation.x = list[4]
        pose.pose.orientation.y = list[5]
        pose.pose.orientation.z = list[6]
        pose.pose.orientation.w = list[7]
        
        return pose

    def joint_states_callback(self, joint_state):

        self.joint_state = joint_state

        return True
    
    def marker_callback(self, msg):
        self.visible_markers = set([marker.name for marker in msg.markers])
        # self.get_logger().info("visible markers = {0}\nmsg = {1}".format(self.visible_markers, msg))

    def save_pose(self, pose_id, frame_id):
        '''
        Looks for the requested frame (the name of an aruco tag or "map") then returns the translation and rotation found by the tf_listener in find_tag as a pose
        in the requested frame.
        '''

        if frame_id in self.visible_markers:
            
            transform = self.get_relative_transform(frame_id,'base_link')
            if transform is not None:
                self.get_logger().info(f'Transform between aruco tag and base_link: {transform}')
            else:
                self.get_logger().info('Could not get transform')
                return False

            pose = PoseStamped()
            pose.header.frame_id = frame_id
            
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z

            pose.pose.orientation.x = transform.transform.rotation.x
            pose.pose.orientation.y = transform.transform.rotation.y
            pose.pose.orientation.z = transform.transform.rotation.z
            pose.pose.orientation.w = transform.transform.rotation.w

            saved_file = open(self.file_path + "/config/saved_poses.json","w")
            self.pose_dict[pose_id.lower()] = self.pose_to_list(pose)
            json.dump(self.pose_dict, saved_file)
            saved_file.close()

            return True

        else: 
            self.get_logger().info("Could not save pose, "+pose_id+" from "+frame_id)
            return False

    def save_pose_callback(self, msg):
        self.save_pose('pose1', 'dock')

    # def go_to_pose(self, pose_id):
    # '''
    # find the pose that save, move_base 
    # '''
    # if pose_id in self.pose_dict:

    #     # self.move_to_pose({'wrist_extension': 0.01})
    #     # self.move_to_pose({'joint_wrist_yaw': 3.3})
    #     # self.move_to_pose({'joint_lift': 0.22})
        
    #     pose_goal = self.list_to_pose(self.pose_dict[pose_id])
    #     tag = pose_goal.header.frame_id

    #     self.tf_buffer = Buffer()

    #     #self.tf_listener = TransformListener(self.tf_buffer, self.node)#TODO

    #     if not self.find_tag(tag):
    #         self.node.get_logger().info("Could not find tag")
    #         return False

    #     map_goal = MoveBase.Goal()

    #     while True:
    #         try:
    #             map_goal.target_pose = self.tf_buffer.transform(pose_goal, 'map', timeout=Duration(seconds=1))
    #             break
    #         except:
    #             if not self.find_tag(tag):
    #                 self.node.get_logger().info("Could not find tag")
    #                 return False

    #     map_goal.target_pose.pose.position.z = 0.0

    #     eul = euler_from_quaternion((map_goal.target_pose.pose.orientation.x, map_goal.target_pose.pose.orientation.y,
    #                                     map_goal.target_pose.pose.orientation.z, map_goal.target_pose.pose.orientation.w))
    #     quat = quaternion_from_euler(0.0, 0.0, eul[2])

    #     map_goal.target_pose.pose.orientation.x = quat[0]
    #     map_goal.target_pose.pose.orientation.y = quat[1]
    #     map_goal.target_pose.pose.orientation.z = quat[2]
    #     map_goal.target_pose.pose.orientation.w = quat[3]

    #     self.node.get_logger().info(f"Sending goal: {map_goal}")
    #     self.client.wait_for_server()
    #     goal_handle = self.client.send_goal(map_goal)
    #     goal_handle.wait_for_result()

    #     self.node.get_logger().info("DONE!")
    #     return True
    
    # else:
    #     self.node.get_logger().info("Pose not found")
    #     return False
    def go_to_pose(self, pose_id):
        if pose_id in self.pose_dict:
            pose_goal = self.list_to_pose(self.pose_dict[pose_id])
            tag_frame = pose_goal.header.frame_id

            # Wait for the transform to be available
            try:
                map_goal = self.tf_buffer.transform(
                    pose_goal,
                    'map',
                    timeout=Duration(seconds=1.0)
                )
            except Exception as ex:
                self.get_logger().error(f"Could not transform pose to 'map' frame: {ex}")
                return False

            # Now, use the BasicNavigator to send the goal
            return self.send_navigation_goal(map_goal)
        else:
            self.get_logger().info("Pose not found")
            return False

    def send_navigation_goal(self, goal_pose):
        # Initialize the navigator
        self.navigator = BasicNavigator()

        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active")

        # Send the navigation goal
        self.navigator.goToPose(goal_pose)

        # Wait for the task to complete
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} meters')
            rclpy.spin_once(self, timeout_sec=0.1)

        # Check the result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Successfully reached the goal!')
            return True
        else:
            self.get_logger().info('Failed to reach the goal')
            return False

    def delete_pose(self, pose_id):
        '''
        Removes a requested pose from the saved pose dictionary
        '''

        if self.pose_dict.has_key(pose_id): 

            saved_file = open(self.file_path + "/config/saved_poses.json","w")
            
            del self.pose_dict[pose_id]
            
            json.dump(self.pose_dict,saved_file)
            saved_file.close()

            print("DELETED ", pose_id)

            return True 
        else: 
            print("Pose not found")
            return False

    def get_relative_transform(self, target_frame, source_frame):

        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())

            return transform 
        except Exception as ex:
            self.get_logger().info(f'Could not transform {source_frame} to {target_frame}: {ex}')
            return None

def main(args=None):
    rclpy.init()
    node = ArucoNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info('interrupt received, so shutting down')

        
    if node is not None:
        node.destroy_node()

    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()
    
    