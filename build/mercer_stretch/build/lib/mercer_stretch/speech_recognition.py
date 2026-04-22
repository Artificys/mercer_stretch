import speech_recognition as sr

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition')
        self.get_logger().info("Speech Recognition Node started")
        
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # Publisher for recognized speech
        self.speech_publisher = self.create_publisher(String, 'speech_text', 10)
        
        # Adjust for ambient noise
        with self.microphone as source:
            self.get_logger().info("Adjusting for ambient noise...")
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
        
        # Start listening in a separate thread
        self.listening_thread = threading.Thread(target=self.listen_continuously)
        self.listening_thread.daemon = True
        self.listening_thread.start()
    
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