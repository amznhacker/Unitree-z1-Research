#!/usr/bin/env python3

"""
Z1 AI Assistant - Voice and natural language control for Z1 robotic arm
Features: Voice commands, natural language processing, intelligent task execution
"""

import rospy
import speech_recognition as sr
import pyttsx3
import threading
import time
import re
from unitree_legged_msgs.msg import MotorCmd
import subprocess
import json

class Z1AIAssistant:
    def __init__(self):
        rospy.init_node("z1_ai_assistant")
        
        # Initialize speech recognition and synthesis
        try:
            self.recognizer = sr.Recognizer()
            self.microphone = sr.Microphone()
            self.tts_engine = pyttsx3.init()
            self.speech_available = True
            
            # Configure TTS
            self.tts_engine.setProperty('rate', 150)
            self.tts_engine.setProperty('volume', 0.8)
            
        except Exception as e:
            rospy.logwarn(f"Speech features not available: {e}")
            self.speech_available = False
        
        # Robot control
        self.joint_pubs = {}
        for i in range(1, 7):
            topic = f"/z1_gazebo/Joint0{i}_controller/command"
            self.joint_pubs[f"Joint0{i}"] = rospy.Publisher(topic, MotorCmd, queue_size=1)
        
        self.gripper_pub = rospy.Publisher("/z1_gazebo/Gripper_controller/command", MotorCmd, queue_size=1)
        
        # Current state
        self.current_positions = {f"Joint0{i}": 0.0 for i in range(1, 7)}
        self.current_positions["Gripper"] = 0.0
        self.listening = False
        
        # Command patterns
        self.command_patterns = {
            'move_up': r'(move|go|lift)\s+(up|higher)',
            'move_down': r'(move|go|lower)\s+(down|lower)',
            'move_left': r'(move|go|turn)\s+(left)',
            'move_right': r'(move|go|turn)\s+(right)',
            'open_gripper': r'(open|release)\s+(gripper|hand|claw)',
            'close_gripper': r'(close|grab|grip)\s+(gripper|hand|claw)',
            'go_home': r'(go|move|return)\s+(home|start|zero)',
            'wave': r'(wave|hello|hi|greet)',
            'demo': r'(demo|demonstration|show)',
            'stop': r'(stop|halt|freeze|emergency)',
            'status': r'(status|state|position|where)',
        }
        
        # Predefined positions
        self.positions = {
            'home': {f"Joint0{i}": 0.0 for i in range(1, 7)},
            'ready': {"Joint01": 0.0, "Joint02": -0.5, "Joint03": 1.0, "Joint04": 0.0, "Joint05": -0.5, "Joint06": 0.0},
            'wave': {"Joint01": 0.5, "Joint02": -0.3, "Joint03": 1.2, "Joint04": 0.0, "Joint05": -0.9, "Joint06": 0.0}
        }
        
        rospy.loginfo("🤖 Z1 AI Assistant initialized")
        if self.speech_available:
            rospy.loginfo("🎤 Voice control ready - say 'Hello Z1' to start")
        else:
            rospy.loginfo("⌨️ Text control ready - type commands")
    
    def speak(self, text):
        """Text-to-speech output"""
        if self.speech_available:
            try:
                self.tts_engine.say(text)
                self.tts_engine.runAndWait()
            except:
                pass
        rospy.loginfo(f"🗣️ Z1: {text}")
    
    def listen_for_wake_word(self):
        """Listen for wake word 'Hello Z1'"""
        if not self.speech_available:
            return False
            
        try:
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=1)
            
            rospy.loginfo("🎤 Listening for wake word 'Hello Z1'...")
            
            with self.microphone as source:
                audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=3)
            
            text = self.recognizer.recognize_google(audio).lower()
            rospy.loginfo(f"👂 Heard: {text}")
            
            if 'hello z1' in text or 'hey z1' in text or 'z1' in text:
                return True
                
        except sr.WaitTimeoutError:
            pass
        except sr.UnknownValueError:
            pass
        except Exception as e:
            rospy.logwarn(f"Speech recognition error: {e}")
        
        return False
    
    def listen_for_command(self):
        """Listen for voice command"""
        if not self.speech_available:
            return None
            
        try:
            rospy.loginfo("🎤 Listening for command...")
            self.speak("Yes, how can I help?")
            
            with self.microphone as source:
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
            
            command = self.recognizer.recognize_google(audio).lower()
            rospy.loginfo(f"👂 Command: {command}")
            return command
            
        except sr.WaitTimeoutError:
            self.speak("I didn't hear anything. Try again.")
        except sr.UnknownValueError:
            self.speak("I didn't understand that. Could you repeat?")
        except Exception as e:
            rospy.logwarn(f"Speech recognition error: {e}")
        
        return None
    
    def parse_command(self, command):
        """Parse natural language command"""
        command = command.lower().strip()
        
        for action, pattern in self.command_patterns.items():
            if re.search(pattern, command):
                return action
        
        # Check for specific positions
        for pos_name in self.positions.keys():
            if pos_name in command:
                return f"go_to_{pos_name}"
        
        # Check for joint-specific commands
        joint_match = re.search(r'joint\s*(\d+)', command)
        if joint_match:
            joint_num = joint_match.group(1)
            if 'up' in command or 'positive' in command:
                return f"move_joint_{joint_num}_positive"
            elif 'down' in command or 'negative' in command:
                return f"move_joint_{joint_num}_negative"
        
        return "unknown"
    
    def execute_command(self, action):
        """Execute parsed command"""
        if action == "move_up":
            self.speak("Moving shoulder up")
            self.move_joint("Joint02", 0.2)
            
        elif action == "move_down":
            self.speak("Moving shoulder down")
            self.move_joint("Joint02", -0.2)
            
        elif action == "move_left":
            self.speak("Turning left")
            self.move_joint("Joint01", 0.3)
            
        elif action == "move_right":
            self.speak("Turning right")
            self.move_joint("Joint01", -0.3)
            
        elif action == "open_gripper":
            self.speak("Opening gripper")
            self.move_joint("Gripper", 0.5)
            
        elif action == "close_gripper":
            self.speak("Closing gripper")
            self.move_joint("Gripper", 0.0)
            
        elif action == "go_home":
            self.speak("Going to home position")
            self.go_to_position("home")
            
        elif action == "wave":
            self.speak("Waving hello!")
            self.wave_sequence()
            
        elif action == "demo":
            self.speak("Starting demonstration")
            self.run_demo()
            
        elif action == "stop":
            self.speak("Emergency stop activated")
            self.emergency_stop()
            
        elif action == "status":
            self.report_status()
            
        elif action.startswith("go_to_"):
            pos_name = action.replace("go_to_", "")
            self.speak(f"Going to {pos_name} position")
            self.go_to_position(pos_name)
            
        elif action.startswith("move_joint_"):
            parts = action.split("_")
            joint_num = parts[2]
            direction = parts[3]
            
            joint_name = f"Joint0{joint_num}"
            delta = 0.2 if direction == "positive" else -0.2
            
            self.speak(f"Moving joint {joint_num}")
            self.move_joint(joint_name, delta)
            
        else:
            self.speak("I don't understand that command. Try saying 'go home', 'wave', or 'open gripper'.")
    
    def move_joint(self, joint_name, delta):
        """Move a specific joint"""
        if joint_name in self.current_positions:
            new_pos = self.current_positions[joint_name] + delta
            
            # Apply limits
            limits = {
                "Joint01": (-1.2, 1.2), "Joint02": (-1.0, 1.0), "Joint03": (0.0, 2.4),
                "Joint04": (-1.2, 1.2), "Joint05": (-1.0, 1.0), "Joint06": (-1.2, 1.2),
                "Gripper": (0.0, 0.6)
            }
            
            if joint_name in limits:
                min_pos, max_pos = limits[joint_name]
                new_pos = max(min_pos, min(max_pos, new_pos))
            
            self.current_positions[joint_name] = new_pos
            
            # Send command
            msg = MotorCmd()
            msg.mode = 10
            msg.q = float(new_pos)
            msg.Kp = 35.0
            msg.Kd = 1.5
            
            if joint_name == "Gripper":
                self.gripper_pub.publish(msg)
            else:
                self.joint_pubs[joint_name].publish(msg)
    
    def go_to_position(self, position_name):
        """Move to predefined position"""
        if position_name in self.positions:
            target_pos = self.positions[position_name]
            
            for joint_name, position in target_pos.items():
                self.current_positions[joint_name] = position
                
                msg = MotorCmd()
                msg.mode = 10
                msg.q = float(position)
                msg.Kp = 35.0
                msg.Kd = 1.5
                
                if joint_name == "Gripper":
                    self.gripper_pub.publish(msg)
                else:
                    self.joint_pubs[joint_name].publish(msg)
    
    def wave_sequence(self):
        """Perform waving motion"""
        # Go to wave position
        self.go_to_position("wave")
        time.sleep(2)
        
        # Wave motion
        for _ in range(3):
            self.move_joint("Joint06", 0.5)
            time.sleep(0.5)
            self.move_joint("Joint06", -0.5)
            time.sleep(0.5)
        
        # Return to neutral wrist
        self.move_joint("Joint06", 0.0)
    
    def run_demo(self):
        """Run demonstration sequence"""
        self.speak("Starting robotic arm demonstration")
        
        # Demo sequence
        positions = ["ready", "wave", "home"]
        
        for pos in positions:
            self.speak(f"Moving to {pos} position")
            self.go_to_position(pos)
            time.sleep(2)
        
        self.speak("Demonstration complete")
    
    def emergency_stop(self):
        """Emergency stop - freeze all joints"""
        for joint_name in self.current_positions.keys():
            msg = MotorCmd()
            msg.mode = 10
            msg.q = float(self.current_positions[joint_name])
            msg.Kp = 100.0  # High stiffness to hold position
            msg.Kd = 5.0
            
            if joint_name == "Gripper":
                self.gripper_pub.publish(msg)
            else:
                self.joint_pubs[joint_name].publish(msg)
    
    def report_status(self):
        """Report current robot status"""
        status_text = "Current robot status: "
        
        for joint_name, position in self.current_positions.items():
            if joint_name == "Gripper":
                status_text += f"Gripper at {position:.2f}. "
            else:
                joint_num = joint_name[-1]
                status_text += f"Joint {joint_num} at {position:.2f} radians. "
        
        self.speak(status_text)
    
    def text_interface(self):
        """Text-based command interface"""
        rospy.loginfo("💬 Text interface ready. Type commands or 'help' for assistance.")
        
        while not rospy.is_shutdown():
            try:
                command = input("Z1> ").strip()
                
                if command.lower() in ['quit', 'exit', 'q']:
                    break
                elif command.lower() == 'help':
                    self.show_help()
                else:
                    action = self.parse_command(command)
                    self.execute_command(action)
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                rospy.logerr(f"Error: {e}")
    
    def show_help(self):
        """Show available commands"""
        help_text = """
Available commands:
- 'go home' - Return to home position
- 'wave' - Wave hello
- 'move up/down/left/right' - Basic movements
- 'open/close gripper' - Gripper control
- 'demo' - Run demonstration
- 'stop' - Emergency stop
- 'status' - Report current position
- 'quit' - Exit program
        """
        print(help_text)
    
    def voice_interface(self):
        """Voice-based command interface"""
        rospy.loginfo("🎤 Voice interface ready. Say 'Hello Z1' to start.")
        
        while not rospy.is_shutdown():
            try:
                if self.listen_for_wake_word():
                    command = self.listen_for_command()
                    if command:
                        action = self.parse_command(command)
                        self.execute_command(action)
                
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                rospy.logerr(f"Voice interface error: {e}")
    
    def run(self):
        """Main execution"""
        self.speak("Z1 AI Assistant ready for commands")
        
        if self.speech_available:
            # Start voice interface in separate thread
            voice_thread = threading.Thread(target=self.voice_interface)
            voice_thread.daemon = True
            voice_thread.start()
            
            rospy.loginfo("🎤 Voice interface running. Press Enter for text interface.")
            input()
        
        # Run text interface
        self.text_interface()
        
        self.speak("Z1 AI Assistant shutting down. Goodbye!")

if __name__ == "__main__":
    try:
        assistant = Z1AIAssistant()
        assistant.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"❌ AI Assistant error: {e}")
        rospy.logerr("💡 Install dependencies: pip3 install SpeechRecognition pyttsx3 pyaudio")