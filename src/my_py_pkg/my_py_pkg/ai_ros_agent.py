#!/usr/bin/env python3

import os  
import sounddevice
import rclpy
from rclpy.node import Node
import subprocess
import re
import time
import requests
import json
import speech_recognition as sr
import pyttsx3
from query_data import query_rag

class AI_ROS_Agent(Node):
    def __init__(self):
        super().__init__('ai_ros_agent')
        self.get_logger().info("AI_ROS_Agent Node started")
        self.input_active = False
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

    def get_ros_command_from_ollama(self, user_query: str):
        self.get_logger().info(f"Preparing RAG-enhanced prompt for: '{user_query}'")

        try:
            rag_result = query_rag(user_query)
            prompt = rag_result["prompt"] if isinstance(rag_result, dict) else rag_result
        except Exception as e:
            self.get_logger().error(f"RAG query failed: {e}")
            return None

        self.get_logger().debug(f"Final prompt sent to Ollama:\n{prompt}")

        data = {
            "model": "llama3",
            "prompt": prompt,
            "stream": True
        }

        try:
            with requests.post("http://localhost:11434/api/generate", json=data, stream=True, timeout=300) as response:
                response.raise_for_status()

                command_output = ""
                self.get_logger().info("Generating response... (thinking)\n")

                for line in response.iter_lines():
                    if line:
                        try:
                            parsed = json.loads(line.decode("utf-8"))
                            chunk = parsed.get("response", "")
                            if chunk.strip():
                                command_output += chunk
                                self.get_logger().info(chunk)
                        except json.JSONDecodeError:
                            self.get_logger().error("Error parsing JSON from Ollama")

                cleaned_command = re.sub(r"<.*?>", "", command_output).strip()

                match = re.search(r"(ros2\s+[^\n\r]+)", cleaned_command)
                if match:
                    return match.group(1).strip()
                else:
                    self.get_logger().error("No valid ROS 2 command found in the RAG-enhanced response")
                    return None

        except requests.exceptions.Timeout:
            self.get_logger().error("Ollama API timeout - Is the server running?")
        except requests.exceptions.RequestException as req_err:
            self.get_logger().error(f"Request error: {req_err}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")
        return None

    def execute_ros_command(self, command):
        if not command:
            self.get_logger().error("No command to execute")
            return False

        command = re.sub(r'\n', ' ', command)
        full_command = f"source /opt/ros/humble/setup.bash && {command}"

        try:
            self.get_logger().info("Executing ROS command")
            self.get_logger().info(f"{command}")

            # 🔧 Changed: run subprocess detached so it won't stop if parent exits
            proc = subprocess.Popen(
                full_command,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setpgrp   # 🔧 detach process group
            )

            time.sleep(1)  # 🔧 small delay to let process start

            # 🔧 Changed: don't wait/block; assume it starts successfully
            return True

        except Exception as e:
            self.get_logger().error(f"Command execution failed: {e}")
            return False

    def handle_user_input(self):
        if self.input_mode == "voice":
            user_input = self.listen_to_user()
        else:
            user_input = input("\nEnter your ROS task request: ").strip()

        if not user_input:
            self.get_logger().error("No input received.")
            return

        self.get_logger().info(f"User input received: '{user_input}'")
        self.get_logger().debug("Generating ROS command with Ollama...")

        ros_command = self.get_ros_command_from_ollama(user_input)

        if not ros_command:
            self.get_logger().error("Failed to get ROS command")
            return

        self.get_logger().debug(f"Generated ROS command:\n{ros_command}")
        self.speak_text(ros_command)

        if ros_command.startswith("ros2"):
            self.get_logger().info("Valid ROS command detected")
            if self.execute_ros_command(ros_command):
                self.get_logger().info("Command executed successfully")
                time.sleep(1)  # 🔧 small delay before accepting next input
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
                user_text = recognizer.recognize_google(audio)
                self.get_logger().info(f"You said: {user_text}")
                return user_text
            except sr.UnknownValueError:
                self.get_logger().error("Could not understand audio.")
            except sr.RequestError as e:
                self.get_logger().error(f"Speech recognition error: {e}")
            except Exception as e:
                self.get_logger().error(f"Unexpected error during listening: {e}")
        return None

    def speak_text(self, text):
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)
        try:
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f"error: {e}")


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
