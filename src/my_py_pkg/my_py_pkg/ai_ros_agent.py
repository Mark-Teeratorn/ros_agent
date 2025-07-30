#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
import subprocess
import re
import time
import requests
import json
import speech_recognition as sr
import pyttsx3
from .query_data import query_rag

class AI_ROS_Agent(Node):
    def __init__(self):
        super().__init__('ai_ros_agent')
        self.get_logger().info("AI_ROS_Agent Node started")
        self.input_active = False
        self.current_process = None
        self.tts_engine = pyttsx3.init()  # Initialize once to prevent reference errors
        self.tts_engine.setProperty('rate', 150)
        self.input_mode = self.select_input_mode()
        self.create_timer(0.1, self.check_for_input)

    def select_input_mode(self):
        while True:
            mode = input("Select input mode: [v]oice or [t]ext? ").strip().lower()
            if mode in ['v', 'voice']:
                return 'voice'
            elif mode in ['t', 'text']:
                return 'text'
            else:
                print("Invalid input. Please type 'v' or 't'.")

    def check_for_input(self):
        if not self.input_active:
            self.input_active = True
            try:
                self.handle_user_input()
            finally:
                self.input_active = False

    def get_ros_command_from_openrouter(self, user_query: str):
        api_key = os.environ.get("OPENROUTER_API_KEY")
        if not api_key:
            self.get_logger().error("Missing OPENROUTER_API_KEY environment variable")
            return None

        rag_prompt = query_rag(user_query)

        data = {
            "model": "meta-llama/llama-3.1-405b-instruct",
            "messages": [
                {"role": "system", "content": "You are a strict command generator for ROS 2. "
                                             "Output only the command line, no explanations or markdown."},
                {"role": "user", "content": rag_prompt}
            ]
        }

        try:
            response = requests.post(
                url="https://openrouter.ai/api/v1/chat/completions",
                headers={
                    "Authorization": f"Bearer {api_key}",
                    "Content-Type": "application/json"
                },
                data=json.dumps(data),
                timeout=60
            )
            response.raise_for_status()
            result = response.json()
            answer = result["choices"][0]["message"]["content"].strip()

            match = re.search(r"(ros2\s+[^\n\r]+)", answer)
            if match:
                return match.group(1).strip()
            else:
                self.get_logger().error(f"No valid ROS 2 command found: {answer}")
                return None

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"OpenRouter request failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
        return None

    def execute_ros_command(self, command):
        if not command:
            self.get_logger().error("No command to execute")
            return False

        confirm = input(f"\nAbout to run:\n{command}\nProceed? (y/n): ").strip().lower()
        if confirm not in ['y', 'yes']:
            self.get_logger().info("Command cancelled by user.")
            return False

        if self.current_process and self.current_process.poll() is None:
            self.get_logger().info("Stopping previous command...")
            self.stop_running_process()

        full_command = f"source /opt/ros/humble/setup.bash && source ~/ired_ws/install/setup.bash && {command}"

        try:
            self.get_logger().info(f"Executing ROS command: {command}")
            process = subprocess.Popen(
                ["bash", "-c", full_command],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                env=os.environ.copy()
            )
            self.current_process = process

            # Instead of blocking on communicate(), just keep it running
            self.get_logger().info("Process started. Type 'stop' to cancel it.")
            return True

        except Exception as e:
            self.get_logger().error(f"Command execution failed: {e}")
            return False


    def stop_running_process(self):
        if self.current_process and self.current_process.poll() is None:
            self.get_logger().info("Stopping running process...")
            self.current_process.terminate()
            try:
                self.current_process.wait(timeout=5)
                self.get_logger().info("Process stopped successfully.")
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Force killing process...")
                self.current_process.kill()
                self.get_logger().info("Process killed.")
            self.current_process = None
        else:
            self.get_logger().info("No running process to stop.")

    def handle_user_input(self):
        if self.input_mode == "voice":
            user_input = self.listen_to_user()
        else:
            user_input = input("\nEnter your ROS task request (type 'stop' to cancel running command): ").strip()

        if not user_input:
            self.get_logger().error("No input received.")
            return

        if user_input.lower() == "stop":
            self.stop_running_process()
            return

        self.get_logger().info(f"User input received: '{user_input}'")
        ros_command = self.get_ros_command_from_openrouter(user_input)

        if not ros_command:
            self.get_logger().error("Failed to get ROS command")
            return

        self.get_logger().info(f"Generated ROS command:\n{ros_command}")
        self.speak_text(ros_command)

        if ros_command.startswith("ros2"):
            if self.execute_ros_command(ros_command):
                self.get_logger().info("Command started successfully; now running in background.")
                time.sleep(1)
            else:
                self.get_logger().error("Command execution failed")
        else:
            self.get_logger().info("AI response was not a ROS 2 command")


    def listen_to_user(self):
        recognizer = sr.Recognizer()
        with sr.Microphone() as source:
            self.get_logger().info("Listening... Please speak.")
            try:
                audio = recognizer.listen(source, timeout=30, phrase_time_limit=15)
                text = recognizer.recognize_google(audio)
                self.get_logger().info(f"You said: {text}")
                return text
            except sr.UnknownValueError:
                self.get_logger().error("Could not understand audio.")
            except sr.RequestError as e:
                self.get_logger().error(f"Speech recognition error: {e}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error during listening: {e}")
        return None

    def speak_text(self, text):
        try:
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AI_ROS_Agent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Operation cancelled by user")
    except Exception as e:
        node.get_logger().error(f"Unexpected error in main(): {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
