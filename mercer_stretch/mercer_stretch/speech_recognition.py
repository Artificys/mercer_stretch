import speech_recognition as sr
from google import genai
from dotenv import load_dotenv
import os
from playsound import playsound
import subprocess


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
        self.on_tour = False
        
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
            If a user asks for a tour of the lab, respond with $CMD_TOUR. Do not start responses with $CMD unless a specified command is prompted.
            If a user tells you to home the robot and includes the word execute in their prompt, respond with $CMD_HOME.
            If a user tells you to stow the robot and includes the word execute in their prompt, respond with $CMD_STOW.
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
        self._stop_loading = False
    

    def listen_continuously(self):
        if self.on_tour: return
        # listens continously and processes audio
        # if speech is recognized, it is sent to the Gemini API
        with self.microphone as source:
            self.get_logger().info("Listening for speech...")
            while rclpy.ok():
                try:
                    audio = self.recognizer.listen(source, timeout=3, phrase_time_limit=10)
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
        # calls text to speech service from mercer text to speech node
        self.tts_request.message = message
        self.future = self.cli.call_async(self.tts_request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def execute_command(self, command):
        # for user voice commands given to the robot
        if command == "TOUR":
            self.on_tour = True
            self.get_logger().info("Executing tour command")
            self.request_text_to_speech("Certainly! We will now begin the tour")
            # Launch mercer_nav node when tour command runs
            result = subprocess.run(
                ["ros2", "run", "mercer_stretch", "mercer_nav"],
                capture_output=True, text=True
            )
            if result.returncode == 0:
                self.get_logger().info("mercer_nav launched successfully")
            else:
                self.get_logger().error(f"Failed to launch mercer_nav: {result.stderr}")

        elif command == "HOME":
            self.get_logger().info("Executing home command")
            self.request_text_to_speech("Homing")
            result = subprocess.run(["stretch_free_robot_process.py"], capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info("Robot process freed")
            result = subprocess.run(["stretch_robot_home.py"], capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info("Robot homed")

        elif command == "STOW":
            self.get_logger().info("Executing stow command")
            self.request_text_to_speech("Stowing")
            result = subprocess.run(["ros2", "service", "call", "/stow_the_robot", "std_srvs/srv/Trigger", "{}"], capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info("Robot stowed")


    def consult_the_devil(self, message):
        self._stop_loading = False
        # sends user message to Gemini API and gets response. If command is received, execute command
        # otherwise, response is sent to text to speech node

        # start audio playback in background thread
        self._stop_loading = False
        audio_thread = threading.Thread(target=self._play_loading_audio, daemon=True)
        audio_thread.start()
        
        try:
            response = self.chat.send_message(message)
            self._stop_loading = True
            if response.text[:5] == "$CMD_":
                command = response.text[5:]
                self.get_logger().info(f"Received command: {command}")
                self.execute_command(command)
                return
        except Exception as e:
            self.get_logger().error(f"Error communicating with Gemini API: {e}")
            result = self.request_text_to_speech("Looks like an error occurred. Contact Zach at N O B L E Z @ R P I . E D U and tell him to get on it.")
            return
        else:
            self.get_logger().info(f"Recieved response: {response.text}")
            result = self.request_text_to_speech(response.text)
        return result

    def _play_loading_audio(self):
        """Play loading audio in a loop until stopped"""
        audio_path = os.path.join(package_share_directory, "audio", "waiting.mp3")
        while not self._stop_loading:
            try:
                playsound(audio_path, block=True)
            except Exception as e:
                self.get_logger().error(f"Error playing audio: {e}")
                break
            if self._stop_loading:
                break


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