
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from stretch_nav2.robot_navigator import BasicNavigator, TaskResult
from mercer_interfaces.action import MercerAudio

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration
import json
import simpleaudio as sa
    
class MercerAudioNode(Node):
    def __init__(self):
        
        super().__init__('mercer_audio')
        node_name = self.get_name()
        self.get_logger().info("{0} started".format(node_name))

        self.audio_server = ActionServer(self, MercerAudio, 'mercer_audio_server', self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info("Received goal request")
        result = MercerAudio.Result()
        file_path = goal_handle.request.file_path

        try:
            wave_obj = sa.WaveObject.from_wave_file(file_path)
            play_obj = wave_obj.play()
            play_obj.wait_done()
            result.finished = MercerAudio.Result.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Failed to play audio: {e}")
            result.finished = MercerAudio.Result.FAILURE

        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MercerAudioNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')

        
    if node is not None:
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()