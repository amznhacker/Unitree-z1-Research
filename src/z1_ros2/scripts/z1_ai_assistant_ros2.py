#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import speech_recognition as sr
import pyttsx3
import threading
import re

class Z1AIAssistantROS2(Node):
    def __init__(self):
        super().__init__('z1_ai_assistant_ros2')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/z1/cmd_vel', 10)
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        
        # Speech recognition and synthesis
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.tts_engine = pyttsx3.init()
        
        # Robot state
        self.current_joints = [0.0] * 6
        self.is_listening = False
        
        # Voice commands mapping
        self.commands = {
            'home': self.go_home,
            'wave': self.wave_gesture,
            'stop': self.emergency_stop,
            'up': lambda: self.move_joint('shoulder', 0.5),
            'down': lambda: self.move_joint('shoulder', -0.5),
            'left': lambda: self.move_joint('base', 0.5),
            'right': lambda: self.move_joint('base', -0.5),
            'forward': lambda: self.move_joint('elbow', 0.5),
            'backward': lambda: self.move_joint('elbow', -0.5)
        }
        
        self.get_logger().info('Z1 AI Assistant ROS2 initialized')
        self.speak("Z1 AI Assistant ready. Say 'hello robot' to start.")
        
        # Start listening thread
        self.listen_thread = threading.Thread(target=self.listen_continuously)
        self.listen_thread.daemon = True
        self.listen_thread.start()
    
    def joint_callback(self, msg):
        """Update current joint positions"""
        if len(msg.position) >= 6:
            self.current_joints = list(msg.position[:6])
    
    def speak(self, text):
        """Text-to-speech output"""
        self.get_logger().info(f'Speaking: {text}')
        self.tts_engine.say(text)
        self.tts_engine.runAndWait()
    
    def listen_continuously(self):
        """Continuous speech recognition"""
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
        
        while rclpy.ok():
            try:
                with self.microphone as source:
                    # Listen for wake word or commands
                    audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=3)
                
                try:
                    command = self.recognizer.recognize_google(audio).lower()
                    self.get_logger().info(f'Heard: {command}')
                    
                    # Wake word detection
                    if 'hello robot' in command or 'hey robot' in command:
                        self.is_listening = True
                        self.speak("Yes, how can I help you?")
                        continue
                    
                    # Process commands if listening
                    if self.is_listening:
                        self.process_voice_command(command)
                        
                except sr.UnknownValueError:
                    pass  # Ignore unrecognized speech
                except sr.RequestError as e:
                    self.get_logger().error(f'Speech recognition error: {e}')
                    
            except sr.WaitTimeoutError:
                pass  # Continue listening
            except Exception as e:
                self.get_logger().error(f'Listening error: {e}')
    
    def process_voice_command(self, command):
        """Process recognized voice commands"""
        # Stop listening after processing
        self.is_listening = False
        
        # Check for direct command matches
        for keyword, action in self.commands.items():
            if keyword in command:
                self.speak(f"Executing {keyword}")
                action()
                return
        
        # Complex command parsing
        if 'move' in command:
            if 'arm up' in command or 'raise arm' in command:
                self.speak("Raising arm")
                self.move_joint('shoulder', 1.0)
            elif 'arm down' in command or 'lower arm' in command:
                self.speak("Lowering arm")
                self.move_joint('shoulder', -1.0)
            elif 'turn left' in command:
                self.speak("Turning left")
                self.move_joint('base', 1.0)
            elif 'turn right' in command:
                self.speak("Turning right")
                self.move_joint('base', -1.0)
        
        elif 'status' in command or 'how are you' in command:
            self.speak(f"All systems operational. Current joint positions: {[round(j, 2) for j in self.current_joints]}")
        
        elif 'goodbye' in command or 'stop listening' in command:
            self.speak("Goodbye! Say hello robot to wake me up again.")
        
        else:
            self.speak("I didn't understand that command. Try saying home, wave, stop, or move commands.")
    
    def move_joint(self, joint_name, velocity):
        """Move specific joint with voice command"""
        twist = Twist()
        
        if joint_name == 'base':
            twist.angular.z = velocity
        elif joint_name == 'shoulder':
            twist.linear.y = velocity
        elif joint_name == 'elbow':
            twist.linear.z = velocity
        elif joint_name == 'wrist':
            twist.angular.x = velocity
        
        self.cmd_pub.publish(twist)
        
        # Stop after brief movement
        self.create_timer(0.5, lambda: self.cmd_pub.publish(Twist()))
    
    def go_home(self):
        """Move to home position"""
        self.get_logger().info('Moving to home position')
        # Send home command (implementation depends on controller)
    
    def wave_gesture(self):
        """Perform wave gesture"""
        self.get_logger().info('Performing wave gesture')
        # Implement wave sequence
    
    def emergency_stop(self):
        """Emergency stop all movement"""
        self.get_logger().info('Emergency stop activated')
        self.cmd_pub.publish(Twist())  # Zero velocity
        self.speak("Emergency stop activated")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        ai_assistant = Z1AIAssistantROS2()
        rclpy.spin(ai_assistant)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()