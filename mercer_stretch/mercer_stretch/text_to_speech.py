import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from voicebox.tts import ESpeakNG, ESpeakConfig
from voicebox import SimpleVoicebox
from voicebox.effects import Vocoder, Normalize
from mercer_interfaces.srv import TextToSpeech

class TextToSpeechNode(Node):
    def __init__(self):
        super().__init__('text_to_speech')
        node_name = self.get_name()
        self.get_logger().info("{0} started".format(node_name))

        self.srv = self.create_service(TextToSpeech, "text_to_speech", self.text_to_speech_callback)

        engine_config = ESpeakConfig(speed = 120, pitch = 30, voice = "en-us")
        engine = ESpeakNG(config=engine_config)
        self.voicebox = SimpleVoicebox(engine, effects=[Vocoder.build(), Normalize()])
        self.voicebox.say("text to speech node is online")

    
    def text_to_speech_callback(self, request, response):
        self.get_logger().info(f"text to speech service got request: {request.message}")
        self.voicebox.say(request.message + " ")
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