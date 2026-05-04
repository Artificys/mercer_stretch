"""
I had to do some annoying things to get piper to work. If you're ever setting this up on another robot, here's what to do:
1. python3 -m venv ~/piper_venv
2. source ~/piper_venv/bin/activate
3. pip install piper-tts
4. in ~/piper_venv/models run python3 -m piper.download_voices en_us-john-medium
"""
import os
import sys
venv_site = os.path.expanduser("~/piper_venv/lib/python3.10/site-packages")
sys.path.insert(0, venv_site)


import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from piper import PiperVoice
import simpleaudio as sa
from mercer_interfaces.srv import TextToSpeech

class TextToSpeechNode(Node):
    def __init__(self):
        super().__init__('text_to_speech')
        node_name = self.get_name()
        self.get_logger().info("{0} started".format(node_name))

        self.srv = self.create_service(TextToSpeech, "text_to_speech", self.text_to_speech_callback)

        try:
            self.voice = PiperVoice.load(os.path.expanduser("~/piper_venv/models/en_US-john-medium.onnx"))
        except Exception as e:
            self.get_logger().error(f"Failed to load Piper voice: {e}")
            self.voice = None
            return

        for chunk in self.voice.synthesize("Text to speech node is online"):
            # Play audio directly without saving to file
            audio_obj = sa.play_buffer(
                chunk.audio_int16_bytes,
                num_channels=1,
                bytes_per_sample=2,
                sample_rate=chunk.sample_rate
            )
            audio_obj.wait_done()

    
    def text_to_speech_callback(self, request, response):
        self.get_logger().info(f"text to speech service got request: {request.message.strip()}")
        if self.voice is None:
            self.get_logger().error("Voice not loaded")
            response.result = -1
            return response
        for chunk in self.voice.synthesize(request.message):
            # Play audio directly without saving to file
            audio_obj = sa.play_buffer(
                chunk.audio_int16_bytes,
                num_channels=1,
                bytes_per_sample=2,
                sample_rate=chunk.sample_rate
            )
            audio_obj.wait_done()
        response.result = 0
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TextToSpeechNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down text to speech Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()