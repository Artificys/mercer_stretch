import speech_recognition as sr
from google import genai
from dotenv import load_dotenv
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time
from ament_index_python.packages import get_package_share_directory

from mercer_interfaces.srv import TextToSpeech

package_share_directory = get_package_share_directory("mercer_stretch")
dotenv_path = os.path.join(package_share_directory, ".env")
load_dotenv(dotenv_path)

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition')
        self.get_logger().info("Speech Recognition Node started")
        self.get_logger().info(f"looking for .env at {dotenv_path}")
        key = str(os.getenv("GEMINI_API_KEY"))
        self.get_logger().info(f"Got API key beginning in {key[:5]}")
        
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Publisher for recognized speech
        self.speech_publisher = self.create_publisher(String, 'speech_text', 10)
        self.client = genai.Client(api_key = key)
        self.chat = self.client.chats.create(model="gemma-4-31b-it")
        test_response = self.chat.send_message("Testing. Please respond with \"Online\"")
        self.get_logger().info(f"Received response to test prompt: {test_response.text}")
        if len(test_response.text) > 0:
            response = self.chat.send_message("""You are a Stretch RE1 Robot located in the Mercer Lab, a lab for the Department of Electrical, Computer, and Systems Engineering at Rensselaer Polytechnic Institute.
            Soldering kits are available if you ask the storeroom worker. The storeroom is located at the entrance. Benchtop equipment including oscilloscopes, power supplies, and function generators are available at the worktables in the back. There are PCB printers on the right side. Resistors and some 74 series chips are available on the table by the PCB printers.
            Spools of wire and jumper cables are available at the back of the lab by the patent wall.
            Respond with \"understood\"
            """)
            self.get_logger().info(f"Received response to information prompt: {response.text}")
        
        # Adjust for ambient noise
        with self.microphone as source:
            self.get_logger().info("Adjusting for ambient noise...")
            self.recognizer.adjust_for_ambient_noise(source, duration=3)
            self.get_logger().info(f"Set energy threshold at {self.recognizer.energy_threshold}, which will be increased 20%")
        self.recognizer.energy_threshold *= 1.2

        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.dynamic_energy_adjustment_damping = 0.15
        self.recognizer.dynamic_energy_ratio = 1.8
        
        # Start listening in a separate thread
        self.listening_thread = threading.Thread(target=self.listen_continuously)
        self.listening_thread.daemon = True
        self.listening_thread.start()

        self.cli = self.create_client(TextToSpeech, "text_to_speech")
        if not self.cli.wait_for_service(timeout_sec = 5.0):
            self.get_logger().warn("Text to speech service offline")

        self.tts_request = TextToSpeech.Request()
    
    def listen_continuously(self):
        with self.microphone as source:
            self.get_logger().info("Listening for speech...")
            while rclpy.ok():
                try:
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=10)
                    self.get_logger().info("Processing audio...")
                    
                    # Recognize speech
                    text = self.recognizer.recognize_google(audio)
                    self.get_logger().info(f"Recognized: {text}")
                    self.consult_the_devil(text)
                    
                    # Publish the recognized text
                    msg = String()
                    msg.data = text
                    self.speech_publisher.publish(msg)
                    
                except sr.WaitTimeoutError:
                    # Timeout, continue listening
                    pass
                except sr.UnknownValueError:
                    self.get_logger().warn("Could not understand audio")
                except sr.RequestError as e:
                    self.get_logger().error(f"API request error: {e}")
                except Exception as e:
                    self.get_logger().error(f"Unexpected error: {e}")

    def request_text_to_speech(self, message):
        self.tts_request.message = message
        self.future = self.cli.call_async(self.tts_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def consult_the_devil(self, message):
        response = self.chat.send_message(message)
        self.get_logger().info(f"Recieved response: {response.text}")
        result = self.request_text_to_speech(response.text)



def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Speech Recognition Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()