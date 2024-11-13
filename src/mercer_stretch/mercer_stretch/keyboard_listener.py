import time
import sys
import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.timer import Timer
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from stretch_core.keyboard import KBHit



controls = '''
---- Controls ----
                  
Base Controls     
-------------     
  w               
a s d             
                  
Head Controls     
-------------     
  8               
4 2 6             
                  
Lift Controls     
-------------     
 - +              
                  
Arm Controls      
-------------     
7   9 Extend      
1   3 Wrist       
                  
Step Size         
-------------     
b: Small          
n: Medium         
m: Large          
                  
q to Quit         
------------------'''

class KeyboardListener(Node):

    def __init__(self):
        super().__init__('keyboard_listener')
        self.kb = KBHit()

        self.step_size = 'medium'
        self.deltas = {
            'small': {
                'rotation': np.radians(5),
                'translation': 0.01
            },
            'medium': {
                'rotation': np.radians(10),
                'translation': 0.05
            }, 
            'large': {
                'rotation': np.radians(15),
                'translation': 0.1
            }
        }

        print(controls)

        self.translate_delta = self.deltas[self.step_size]['translation']
        self.rotate_delta = self.deltas[self.step_size]['rotation']

        self.declare_parameter('update_rate', 50)
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().integer_value
        self.key_timer = self.create_timer(1/self.update_rate, self.get_command)

        # self.declare_parameter('buffer_time', 10)
        # self.buffer_time = self.get_parameter('buffer_time').get_parameter_value().integer_value
        # self.buffer_timer = self.create_timer(self.buffer_time, self.buffer_callback)

        self.joint_state = JointState()
        self.joint_states_sub = self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        self.joint_states_sub

        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        server_reached = self.trajectory_client.wait_for_server(timeout_sec=60.0)
        if not server_reached:
            self.get_logger().error('Unable to connect to arm action server. Timeout exceeded.')
            sys.exit()

        self.status_pub = self.create_publisher(String, '/keyboard_listener/status', 1)
        self.status_str = String()
        self.status_str.data = 'inactive'



    # def buffer_callback(self):
    #     self.buffer_timer.destroy()
    #     self.get_logger().info('No key presses detected for 10 seconds since creation, so destroying node')
    #     self.self_state_pub.publish(Bool(data=False))
    #     sys.exit()

    def joint_states_callback(self, joint_state):
        self.joint_state = joint_state

    def get_command(self):
        command = None
        c = None

        if self.kb.kbhit(): # Returns True if any key pressed
            c = self.kb.getch()
            self.status_str.data = 'active'
            self.status_pub.publish(self.status_str)


        ####################################################
        ## BASIC KEYBOARD TELEOPERATION COMMANDS
        ####################################################

        match c:
        #Base Controls
            case 'w': 
                command = {'joint': 'translate_mobile_base', 'inc': self.translate_delta}
            case 'a':
                command = {'joint': 'rotate_mobile_base', 'inc': self.rotate_delta}
            case 's':
                command = {'joint': 'translate_mobile_base', 'inc': -self.translate_delta}
            case 'd':
                command = {'joint': 'rotate_mobile_base', 'inc': -self.rotate_delta}
        #Lift Controls
            case '+':
                command = {'joint': 'joint_lift', 'delta': self.translate_delta}
            case '-':
                command = {'joint': 'joint_lift', 'delta': -self.translate_delta}
        #Head Controls
            case '8':
                command = {'joint': 'joint_head_tilt', 'delta': (2.0 * self.rotate_delta)}
            case '2':
                command = {'joint': 'joint_head_tilt', 'delta': -(2.0 * self.rotate_delta)}
            case '6':
                command = {'joint': 'joint_head_pan', 'delta': (2.0 * self.rotate_delta)}
            case '4':
                command = {'joint': 'joint_head_pan', 'delta': -(2.0 * self.rotate_delta)}
        #Arm Controls
            case '9':
                command = {'joint': 'wrist_extension', 'delta': self.translate_delta}
            case '1':
                command = {'joint': 'joint_wrist_yaw', 'delta': self.rotate_delta}
            case '3':
                command = {'joint': 'joint_wrist_yaw', 'delta': -self.rotate_delta}
            case '7':
                command = {'joint': 'wrist_extension', 'delta': -self.translate_delta}
        #Step Size
            case 'm':
                self.get_logger().info('process_keyboard.py: changing to BIG step size')
                self.step_size = 'big'
                self.translate_delta = self.deltas[self.step_size]['translation']
                self.rotate_delta = self.deltas[self.step_size]['rotation']
            case 'n':
                self.get_logger().info('process_keyboard.py: changing to MEDIUM step size')
                self.step_size = 'medium'
                self.translate_delta = self.deltas[self.step_size]['translation']
                self.rotate_delta = self.deltas[self.step_size]['rotation']
            case 'b':
                self.get_logger().info('process_keyboard.py: changing to SMALL step size')
                self.step_size = 'small'
                self.translate_delta = self.deltas[self.step_size]['translation']
                self.rotate_delta = self.deltas[self.step_size]['rotation']
            
            case 't':
                #Print Transform between aruco tag and base_link
                transform = self.get_relative_transform('thomas_test','base_link')
                if transform is not None:
                    self.get_logger().info(f'Transform between aruco tag and base_link: {transform}')
                else:
                    self.get_logger().info('Could not get transform')

        #Quit
            case 'q':
                self.get_logger().info('keyboard_teleop exiting...')
                self.get_logger().info('Received quit character (q), so exiting')
                self.status_str.data = 'inactive'
                self.status_pub.publish(self.status_str)
                sys.exit()
            case _:
                pass

        ####################################################

        self.send_command(command)

    def get_relative_transform(self, target_frame, source_frame):
        tf_buffer = Buffer()
        listener = TransformListener(tf_buffer, self)

        try:
            transform = tf_buffer.lookup_transform(target_frame, source_frame, Time())
            return transform 
        except Exception as ex:
            self.get_logger().info(f'Could not transform {source_frame} to {target_frame}: {ex}')
            return None

    def send_command(self, command):
        joint_state = self.joint_state
        if (joint_state is not None) and (command is not None):
            point = JointTrajectoryPoint()
            duration = Duration(seconds=0.0)
            point.time_from_start = duration.to_msg()
            trajectory_goal = FollowJointTrajectory.Goal()
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]

            # for base joints
            if 'inc' in command:
                new_value = command['inc']

            # for non-base joints
            elif 'delta' in command:
                print(joint_state)
                joint_index = joint_state.name.index(joint_name)
                joint_value = joint_state.position[joint_index]
                new_value = joint_value + command['delta']
            
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            self.trajectory_client.send_goal_async(trajectory_goal)

def main():
    try:

        rclpy.init()
        node = KeyboardListener()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')
    
    if node is not None:
        node.kb.set_normal_term()
        node.destroy_node()

    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()

