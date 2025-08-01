#!/usr/bin/env python3

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
        self.create_timer(0.1, self.check_for_input)
        self.input_active = False



    def check_for_input(self):
        if not self.input_active:
            self.input_active = True
            try:
                self.handle_user_input()
            finally:
                self.input_active = False

    def get_ros_command_from_ollama(self, user_query: str):
        self.get_logger().info(f"Preparing RAG-enhanced prompt for: '{user_query}'")

        # Step 1: Query RAG to get relevant context + build prompt
        try:
            prompt = query_rag(user_query)
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

                # Accept any ros2 command
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

            proc = subprocess.Popen(
                full_command,
                shell=True,
                executable="/bin/bash",
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            try:
                stdout, stderr = proc.communicate(timeout=5)  # Collect output
            except subprocess.TimeoutExpired:
                self.get_logger().warning("Command timeout, killing process")
                proc.kill()
                stdout, stderr = proc.communicate()

            # Print stdout and stderr if they exist
            stdout_decoded = stdout.decode('utf-8').strip()
            stderr_decoded = stderr.decode('utf-8').strip()

            if stdout_decoded:
                print("\n STDOUT:\n" + stdout_decoded)
                self.get_logger().info(f"STDOUT:\n{stdout_decoded}")
            if stderr_decoded:
                print("\n STDERR:\n" + stderr_decoded)
                self.get_logger().warn(f"STDERR:\n{stderr_decoded}")

            return proc.returncode == 0

        except Exception as e:
            self.get_logger().error(f"Command execution failed: {e}")
            return False

    def handle_user_input(self):
        user_input = self.listen_to_user()
        if not user_input:
            self.get_logger().error("No voice input received. Exiting.")
            return

        self.get_logger().info(f"User input received: '{user_input}'")
        self.get_logger().debug("Generating ROS command or answer with Ollama...")

        ros_command = self.get_ros_command_from_ollama(user_input)

        if not ros_command:
            self.get_logger().error("Failed to get response from Ollama")
            return

        self.get_logger().debug(f"Generated command or answer:\n{ros_command}")
        self.speak_text(ros_command)

        if ros_command.startswith("ros2"):
            self.get_logger().info("Valid ROS command detected")
            if self.execute_ros_command(ros_command):
                self.get_logger().info("Command executed successfully")
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