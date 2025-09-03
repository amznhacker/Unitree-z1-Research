#!/usr/bin/env python3
"""
Z1 Visual Programmer - Scratch-like GUI for Z1 Programming
"""
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import json, math, threading, time
import rospy
from unitree_legged_msgs.msg import MotorCmd

class Z1Block:
    def __init__(self, block_type, params=None):
        self.type = block_type
        self.params = params or {}

class Z1VisualProgrammer:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Z1 Visual Programmer")
        self.root.geometry("1200x800")
        
        self.ros_initialized = False
        self.pubs = {}
        self.joints = ["Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06", "Gripper"]
        self.program_blocks = []
        self.running = False
        
        self.limits = {
            "Joint01": (-1.2, 1.2), "Joint02": (-1.0, 1.0), "Joint03": (0.0, 2.4),
            "Joint04": (-1.2, 1.2), "Joint05": (-1.0, 1.0), "Joint06": (-1.2, 1.2),
            "Gripper": (0.0, 0.8)
        }
        
        self.poses = {
            "Home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8],
            "Forward": [0.0, -0.5, 1.2, 0.0, -0.7, 0.0, 0.8],
            "Up": [0.0, 0.8, 0.4, 0.0, 0.4, 0.0, 0.8],
            "Table": [0.0, -0.8, 1.5, 0.0, -0.7, 0.0, 0.8]
        }
        
        self.setup_gui()
        self.init_ros()
    
    def init_ros(self):
        try:
            rospy.init_node("z1_visual_programmer", anonymous=True)
            for joint in self.joints:
                topic = f"/z1_gazebo/{joint}_controller/command"
                self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=10)
            self.ros_initialized = True
            self.status_label.config(text="🟢 Connected", fg="green")
        except:
            self.status_label.config(text="🔴 Disconnected", fg="red")
    
    def setup_gui(self):
        # Toolbar
        toolbar = tk.Frame(self.root, bg="lightgray", height=40)
        toolbar.pack(fill=tk.X, side=tk.TOP)
        
        tk.Button(toolbar, text="💾 Save", command=self.save_program).pack(side=tk.LEFT, padx=5)
        tk.Button(toolbar, text="🗑️ Clear", command=self.clear_program).pack(side=tk.LEFT, padx=5)
        
        self.status_label = tk.Label(toolbar, text="🔴 Disconnected", fg="red")
        self.status_label.pack(side=tk.RIGHT, padx=10)
        
        # Block palette
        palette = tk.Frame(self.root, bg="lightblue", width=250)
        palette.pack(fill=tk.Y, side=tk.LEFT)
        palette.pack_propagate(False)
        
        tk.Label(palette, text="🧩 Blocks", font=("Arial", 14, "bold"), bg="lightblue").pack(pady=10)
        
        blocks = [
            ("Move to Pose", "move_pose", "lightgreen"),
            ("Open Gripper", "open_gripper", "lightcoral"),
            ("Close Gripper", "close_gripper", "lightcoral"),
            ("Wait", "wait", "yellow"),
            ("Repeat", "repeat", "plum")
        ]
        
        for name, block_type, color in blocks:
            tk.Button(palette, text=name, bg=color, width=20,
                     command=lambda bt=block_type: self.add_block(bt)).pack(pady=2)
        
        # Program area
        program_frame = tk.Frame(self.root, bg="white")
        program_frame.pack(fill=tk.BOTH, expand=True, side=tk.LEFT)
        
        tk.Label(program_frame, text="📝 Program", font=("Arial", 14, "bold"), bg="white").pack(pady=10)
        
        canvas = tk.Canvas(program_frame, bg="white")
        scrollbar = tk.Scrollbar(program_frame, orient="vertical", command=canvas.yview)
        self.program_area = tk.Frame(canvas, bg="white")
        
        self.program_area.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.program_area, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Control panel
        control = tk.Frame(self.root, bg="lightgray", width=200)
        control.pack(fill=tk.Y, side=tk.RIGHT)
        control.pack_propagate(False)
        
        tk.Label(control, text="🎮 Control", font=("Arial", 14, "bold"), bg="lightgray").pack(pady=10)
        
        tk.Button(control, text="▶️ Run", bg="lightgreen", width=15, command=self.run_program).pack(pady=5)
        tk.Button(control, text="⏹️ Stop", bg="lightcoral", width=15, command=self.stop_program).pack(pady=5)
        tk.Button(control, text="🏠 Home", bg="lightyellow", width=15, command=self.go_home).pack(pady=5)
        
        self.status_text = tk.Text(control, height=8, width=25)
        self.status_text.pack(pady=5)
        
        for pose_name in self.poses.keys():
            tk.Button(control, text=pose_name, width=15,
                     command=lambda p=pose_name: self.quick_pose(p)).pack(pady=1)
    
    def add_block(self, block_type):
        block = Z1Block(block_type)
        
        frame = tk.Frame(self.program_area, bg="lightblue", relief="raised", bd=2)
        frame.pack(fill=tk.X, pady=2, padx=10)
        
        if block_type == "move_pose":
            tk.Label(frame, text="🚀 Move to:", bg="lightgreen").pack(side=tk.LEFT)
            pose_var = tk.StringVar(value="Home")
            combo = ttk.Combobox(frame, textvariable=pose_var, values=list(self.poses.keys()), width=10)
            combo.pack(side=tk.LEFT, padx=5)
            block.params["pose"] = pose_var
            
        elif block_type == "wait":
            tk.Label(frame, text="⏰ Wait:", bg="yellow").pack(side=tk.LEFT)
            time_var = tk.StringVar(value="1.0")
            entry = tk.Entry(frame, textvariable=time_var, width=5)
            entry.pack(side=tk.LEFT, padx=5)
            tk.Label(frame, text="sec", bg="yellow").pack(side=tk.LEFT)
            block.params["time"] = time_var
            
        elif block_type == "repeat":
            tk.Label(frame, text="🔄 Repeat:", bg="plum").pack(side=tk.LEFT)
            count_var = tk.StringVar(value="3")
            entry = tk.Entry(frame, textvariable=count_var, width=3)
            entry.pack(side=tk.LEFT, padx=5)
            block.params["count"] = count_var
            
        elif block_type == "open_gripper":
            tk.Label(frame, text="🤏 Open Gripper", bg="lightcoral").pack(side=tk.LEFT)
            
        elif block_type == "close_gripper":
            tk.Label(frame, text="🤏 Close Gripper", bg="lightcoral").pack(side=tk.LEFT)
        
        # Delete button
        tk.Button(frame, text="❌", command=lambda: self.delete_block(frame, block)).pack(side=tk.RIGHT)
        
        self.program_blocks.append(block)
    
    def delete_block(self, frame, block):
        frame.destroy()
        if block in self.program_blocks:
            self.program_blocks.remove(block)
    
    def send_pose(self, positions, duration=2.0):
        if not self.ros_initialized:
            return
        
        start_time = time.time()
        while time.time() - start_time < duration and not rospy.is_shutdown():
            elapsed = time.time() - start_time
            alpha = elapsed / duration
            smooth_alpha = 0.5 * (1 - math.cos(math.pi * alpha))
            
            for i, joint in enumerate(self.joints):
                if i < len(positions):
                    position = positions[i] * smooth_alpha
                    
                    msg = MotorCmd()
                    msg.mode = 10
                    msg.q = float(position)
                    msg.Kp = 35.0
                    msg.Kd = 1.5
                    self.pubs[joint].publish(msg)
            
            time.sleep(0.02)
    
    def execute_block(self, block):
        if block.type == "move_pose":
            pose_name = block.params["pose"].get()
            if pose_name in self.poses:
                self.log(f"Moving to {pose_name}")
                self.send_pose(self.poses[pose_name])
                
        elif block.type == "wait":
            wait_time = float(block.params["time"].get())
            self.log(f"Waiting {wait_time} seconds")
            time.sleep(wait_time)
            
        elif block.type == "open_gripper":
            self.log("Opening gripper")
            current_pose = [0.0] * 6 + [0.8]
            self.send_pose(current_pose, 1.0)
            
        elif block.type == "close_gripper":
            self.log("Closing gripper")
            current_pose = [0.0] * 6 + [0.2]
            self.send_pose(current_pose, 1.0)
    
    def run_program(self):
        if self.running:
            return
        
        self.running = True
        self.log("🚀 Starting program...")
        
        def run_thread():
            try:
                i = 0
                while i < len(self.program_blocks) and self.running:
                    block = self.program_blocks[i]
                    
                    if block.type == "repeat":
                        count = int(block.params["count"].get())
                        repeat_start = i + 1
                        
                        for _ in range(count):
                            if not self.running:
                                break
                            j = repeat_start
                            while j < len(self.program_blocks) and self.running:
                                if self.program_blocks[j].type == "repeat":
                                    break
                                self.execute_block(self.program_blocks[j])
                                j += 1
                        
                        # Skip to end of repeat block
                        while i < len(self.program_blocks) and self.program_blocks[i].type != "repeat":
                            i += 1
                        i += 1
                    else:
                        self.execute_block(block)
                        i += 1
                
                self.log("✅ Program completed!")
            except Exception as e:
                self.log(f"❌ Error: {e}")
            finally:
                self.running = False
        
        threading.Thread(target=run_thread, daemon=True).start()
    
    def stop_program(self):
        self.running = False
        self.log("⏹️ Program stopped")
    
    def go_home(self):
        self.log("🏠 Going home")
        self.send_pose(self.poses["Home"])
    
    def quick_pose(self, pose_name):
        self.log(f"Quick move to {pose_name}")
        self.send_pose(self.poses[pose_name])
    
    def log(self, message):
        self.status_text.insert(tk.END, f"{message}\n")
        self.status_text.see(tk.END)
        self.root.update()
    
    def save_program(self):
        filename = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON files", "*.json")])
        if filename:
            program_data = []
            for block in self.program_blocks:
                block_data = {"type": block.type, "params": {}}
                for key, var in block.params.items():
                    block_data["params"][key] = var.get()
                program_data.append(block_data)
            
            with open(filename, 'w') as f:
                json.dump(program_data, f, indent=2)
            self.log(f"💾 Saved to {filename}")
    
    def clear_program(self):
        for widget in self.program_area.winfo_children():
            widget.destroy()
        self.program_blocks.clear()
        self.log("🗑️ Program cleared")
    
    def run(self):
        self.root.mainloop()

def main():
    app = Z1VisualProgrammer()
    app.run()

if __name__ == "__main__":
    main()