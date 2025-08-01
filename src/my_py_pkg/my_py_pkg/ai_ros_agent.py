#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
import subprocess
import threading
import re
import requests
import json
import speech_recognition as sr
import pyttsx3
from .query_data import query_rag

class AI_ROS_Agent(Node):
    def __init__(self):
        super().__init__('ai_ros_agent')
        self.get_logger().info("AI_ROS_Agent Node started")
        self.current_process = None
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)
        self.input_mode = self.select_input_mode()

        # Start user input thread
        threading.Thread(target=self.input_loop, daemon=True).start()

    def select_input_mode(self):
        while True:
            mode = input("Select input mode: [v]oice or [t]ext? ").strip().lower()
            if mode in ['v', 'voice']:
                return 'voice'
            elif mode in ['t', 'text']:
                return 'text'
            else:
                print("Invalid input. Please type 'v' or 't'.")

    def input_loop(self):
        while rclpy.ok():
            user_input = self.get_user_input()
            if not user_input:
                continue

            if user_input.lower() == "stop":
                self.stop_running_process()
                continue

            self.get_logger().info(f"User input received: '{user_input}'")
            ros_command = self.get_ros_command_from_openrouter(user_input)

            if not ros_command:
                self.get_logger().error("Failed to get ROS command")
                continue

            self.get_logger().info(f"Generated ROS command:\n{ros_command}")
            self.speak_text(ros_command)

            if ros_command.startswith("ros2"):
                if self.execute_ros_command(ros_command):
                    self.get_logger().info("Command started; running in background.")
                else:
                    self.get_logger().error("Command execution failed.")
            else:
                self.get_logger().info("AI response was not a ROS 2 command")

    def get_user_input(self):
        if self.input_mode == "voice":
            return self.listen_to_user()
        else:
            while True:
                user_input = input("\nEnter your ROS task request (type 'stop' to cancel running command): ").strip()
                if user_input:
                    return user_input
                else:
                    print("(Please enter something, or type 'stop')")

    def get_ros_command_from_openrouter(self, user_query: str):
        api_key = os.environ.get("OPENROUTER_API_KEY")
        if not api_key:
            self.get_logger().error("Missing OPENROUTER_API_KEY environment variable")
            return None

        rag_prompt = query_rag(user_query)
        data = {
            "model": "meta-llama/llama-3.1-405b-instruct:free",
            #meta-llama/llama-3.1-405b-instruct:free
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
            answer = response.json()["choices"][0]["message"]["content"].strip()
            match = re.search(r"(ros2\s+[^\n\r]+)", answer)
            if match:
                return match.group(1).strip()
            else:
                self.get_logger().error(f"No valid ROS 2 command found: {answer}")
        except Exception as e:
            self.get_logger().error(f"OpenRouter request failed: {e}")
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
                bufsize=1
            )
            self.current_process = process

            # Start thread to handle output
            self.stream_output(process)
            return True

        except Exception as e:
            self.get_logger().error(f"Command execution failed: {e}")
            return False


    def stream_output(self, process):
        try:
            # Read both stdout and stderr line by line and print immediately
            for line in iter(process.stdout.readline, ''):
                print(f"[stdout] {line.strip()}", flush=True)
            for line in iter(process.stderr.readline, ''):
                print(f"[stderr] {line.strip()}", flush=True)
        except Exception as e:
            self.get_logger().error(f"Error streaming output: {e}")
        finally:
            retcode = process.wait()
            self.get_logger().info(f"Process finished with return code: {retcode}")
            self.current_process = None


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

    def listen_to_user(self):
        recognizer = sr.Recognizer()
        with sr.Microphone() as source:
            self.get_logger().info("Listening... Please speak.")
            try:
                audio = recognizer.listen(source, timeout=30, phrase_time_limit=15)
                text = recognizer.recognize_google(audio)
                self.get_logger().info(f"You said: {text}")
                return text
            except Exception as e:
                self.get_logger().error(f"Voice input error: {e}")
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
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
