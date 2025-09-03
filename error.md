./quick_start.sh z1_bartender.py
[Z1] Cleaning up existing processes...
[INFO] [1756930678.940126, 84.333200]: Shutting down spawner. Stopping and unloading controllers...
[INFO] [1756930678.944806, 84.333400]: Stopping all controllers...
[WARN] [1756930678.946512, 84.333400]: Controller Spawner error while taking down controllers: unable to contact master
[robot_state_publisher-6] killing on exit
[z1_gazebo/controller_spawner-5] killing on exit
[gazebo_gui-3] killing on exit
[gazebo-2] killing on exit
[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
[Z1] Clearing Gazebo cache...
[Z1] Starting Z1 Simulation...
[Z1] Waiting for Gazebo to initialize...
WARNING: Package name "aliengoZ1_description" does not follow the naming conventions. It should start with a lower case letter and only contain lower case letters, digits, underscores, and dashes.
... logging to /home/zero/.ros/log/17897868-8903-11f0-a454-d94781f12508/roslaunch-zero-Alienware-m15-R3-30123.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

WARNING: Package name "aliengoZ1_description" does not follow the naming conventions. It should start with a lower case letter and only contain lower case letters, digits, underscores, and dashes.
xacro: in-order processing became default in ROS Melodic. You can drop the option.
started roslaunch server http://zero-Alienware-m15-R3:45133/

SUMMARY
========

PARAMETERS
 * /gazebo/enable_ros_network: True
 * /robot_description: <?xml version="1....
 * /rosdistro: noetic
 * /rosversion: 1.17.4
 * /use_sim_time: True
 * /z1_gazebo/Gripper_controller/joint: jointGripper
 * /z1_gazebo/Gripper_controller/pid/d: 50.0
 * /z1_gazebo/Gripper_controller/pid/i: 0.0
 * /z1_gazebo/Gripper_controller/pid/p: 500.0
 * /z1_gazebo/Gripper_controller/type: effort_controller...

Warning [Model.cc:216] Non-unique name[gripperStator] detected 2 times in XML children of model with name[z1_gazebo].
[INFO] [1756930683.914532, 0.000000]: Spawn status: SpawnModel: Successfully spawned entity
[INFO] [1756930683.915366, 0.000000]: Unpausing physics
[INFO] [1756930684.004140916]: Loading gazebo_ros_control plugin
[INFO] [1756930684.004206848]: Starting gazebo_ros_control plugin in namespace: /z1_gazebo
[INFO] [1756930684.004490689]: gazebo_ros_control plugin is waiting for model URDF in parameter [/robot_description] on the ROS param server.
[INFO] [1756930684.117780007]: Loaded gazebo_ros_control.
[INFO] [1756930684.261588, 0.143000]: Controller Spawner: Waiting for service controller_manager/switch_controller
[INFO] [1756930684.263689, 0.145000]: Controller Spawner: Waiting for service controller_manager/unload_controller
[INFO] [1756930684.265091, 0.146400]: Loading controller: joint_state_controller
[INFO] [1756930684.270638, 0.152000]: Loading controller: Joint01_controller
[INFO] [1756930684.277932, 0.158800]: Loading controller: Joint02_controller
[INFO] [1756930684.281878, 0.162200]: Loading controller: Joint03_controller
[INFO] [1756930684.285761, 0.165600]: Loading controller: Joint04_controller
[INFO] [1756930684.289712, 0.169200]: Loading controller: Joint05_controller
[INFO] [1756930684.294085, 0.173200]: Loading controller: Joint06_controller
[INFO] [1756930684.298607, 0.177600]: Loading controller: Gripper_controller
[INFO] [1756930684.305286, 0.184000]: Controller Spawner: Loaded controllers: joint_state_controller, Joint01_controller, Joint02_controller, Joint03_controller, Joint04_controller, Joint05_controller, Joint06_controller, Gripper_controller
[INFO] [1756930684.308207, 0.186600]: Started controllers: joint_state_controller, Joint01_controller, Joint02_controller, Joint03_controller, Joint04_controller, Joint05_controller, Joint06_controller, Gripper_controller
[urdf_spawner-4] process has finished cleanly
log file: /home/zero/.ros/log/17897868-8903-11f0-a454-d94781f12508/urdf_spawner-4*.log
[Z1] Waiting for controllers to load...
[SUCCESS] Controllers loaded successfully
[SUCCESS] Gazebo ready! Starting control: z1_bartender.py
Usage: ./quick_start.sh [keyboard|xbox|demo|draw|real]
  keyboard - Keyboard control (default)
  xbox     - Xbox controller
  demo     - Pick and place demo
  draw     - Drawing demo
  real     - Connect to real robot
[INFO] [1756930698.151454, 13.879600]: Z1 Simple Control initialized with standard controllers
Z1 Simple Control Started
Controls:
  WASD = Base/Shoulder movement
  ZE   = Elbow bend/extend
  RF   = Forearm roll
  TG   = Wrist pitch
  YH   = Wrist roll
  Space = Open gripper
  X     = Close gripper
  ESC   = Emergency stop
  Q     = Quit
Positions: Joint01:0.00° Joint02:0.00° Joint03:0.00° Joint04:0.00° Joint05:0.00° Joint06:0.00° Gripper:0.00 


Absolutely—sticking with MotorCmd is the right call if you want sim to mirror the real Z1. Let’s make it move reliably by streaming commands at 50 Hz with the expected mode and gains.

Below is a clean, copy-paste flow. You’ll end with a tiny ROS node that sweeps a joint back and forth using unitree_legged_msgs/MotorCmd.

✅ Plan (3 terminals)

Terminal A: start Gazebo (keep open)

Terminal B: run the MotorCmd sweep node (streams at 50 Hz)

Terminal C: monitor feedback / debug (optional)

1) Terminal A — launch the sim
source ~/unitree_ws/devel/setup.bash
roslaunch unitree_gazebo z1.launch


Make sure Gazebo is playing (unpaused). If unsure:

rosservice call /gazebo/unpause_physics

2) Terminal B — create and run a MotorCmd sweep node

We’ll make a tiny tools package (one-time), add a script, and run it.

# One-time: create a minimal package that depends on rospy and unitree_legged_msgs
cd ~/unitree_ws/src
catkin_create_pkg z1_tools rospy unitree_legged_msgs
mkdir -p z1_tools/scripts

# Create the sweep script
cat > z1_tools/scripts/z1_motorcmd_sweep.py <<'PY'
#!/usr/bin/env python3
import rospy, math, time, argparse
from unitree_legged_msgs.msg import MotorCmd

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--joint", default="Joint01", help="JointXX (01..06) or Gripper")
    parser.add_argument("--amp", type=float, default=0.6, help="amplitude (rad)")
    parser.add_argument("--freq", type=float, default=0.5, help="sine frequency (Hz)")
    parser.add_argument("--mode", type=int, default=10, help="controller mode (try 10, else 5)")
    parser.add_argument("--kp", type=float, default=35.0, help="Kp gain")
    parser.add_argument("--kd", type=float, default=1.5, help="Kd gain")
    args = parser.parse_args()

    topic = "/z1_gazebo/{}_controller/command".format(args.joint)
    rospy.init_node("z1_motorcmd_sweep_"+args.joint.lower())
    pub = rospy.Publisher(topic, MotorCmd, queue_size=10)

    rate = rospy.Rate(50)  # 50 Hz streaming
    t0 = time.time()

    rospy.loginfo("Publishing MotorCmd to %s (mode=%d, Kp=%.1f, Kd=%.1f)", topic, args.mode, args.kp, args.kd)
    rospy.loginfo("Sine sweep: amp=%.2f rad, freq=%.2f Hz", args.amp, args.freq)

    while not rospy.is_shutdown():
        t = time.time() - t0
        q_des = args.amp * math.sin(2*math.pi*args.freq*t)

        msg = MotorCmd()
        msg.mode = args.mode        # try 10 first; if no motion, try 5
        msg.q = q_des               # desired position (rad)
        msg.dq = 0.0                # desired velocity
        msg.tau = 0.0               # ff torque
        msg.Kp = args.kp
        msg.Kd = args.kd

        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()
PY

# Make it executable and build
chmod +x z1_tools/scripts/z1_motorcmd_sweep.py
cd ~/unitree_ws
catkin_make
source devel/setup.bash

# Run a visible sweep on Joint 1 (about ±0.6 rad)
rosrun z1_tools z1_motorcmd_sweep.py --joint Joint01 --amp 0.6 --freq 0.2 --mode 10 --kp 35 --kd 1.5


If there’s still no motion, stop the script (Ctrl-C) and run again with --mode 5, or bump --kp to 45 (keep --kd around 1.5–2.0). Always keep Gazebo unpaused.

Examples:

# Try alternate mode
rosrun z1_tools z1_motorcmd_sweep.py --joint Joint01 --mode 5

# Try stiffer control
rosrun z1_tools z1_motorcmd_sweep.py --joint Joint01 --kp 45 --kd 1.5


Sweep other joints by changing --joint Joint0N or --joint Gripper.

3) Terminal C — verify and monitor (optional)
source ~/unitree_ws/devel/setup.bash

# Time should advance
rostopic echo -n 1 /clock

# Controller state for the joint you’re sweeping
rostopic echo -n 5 /z1_gazebo/Joint01_controller/state

# Overall joint states (position array index 0 is joint1)
rostopic echo -n 5 /z1_gazebo/joint_states


You should see position[0] change smoothly as the sine runs.

Common gotchas & fixes

No motion at all

Make sure Gazebo is unpaused: rosservice call /gazebo/unpause_physics

Confirm the topic type: rostopic info /z1_gazebo/Joint01_controller/command → must be unitree_legged_msgs/MotorCmd

Try --mode 5 instead of 10

Increase --kp gradually (35 → 45). If jittery/oscillatory, reduce --kp or raise --kd slightly.

Tiny movement only

Increase amplitude: --amp 0.8 (stay within safe range, e.g., ≤ ~1.0 rad unless you know limits)

Lower frequency: --freq 0.1 (moves slower but more obviously)

Laggy UI

Gazebo can be heavy; check that you’re on the NVIDIA driver. (You already set this up earlier.)

Why this mirrors the real robot

You’re driving the same message structure (MotorCmd) and a streaming control pattern the hardware expects (continuous updates, correct mode, tuned gains).

Later, when you switch z1_controller to UDP (COMMUNICATION UDP) for the real arm, your high-level behavior and timing assumptions remain valid.

If you want, I can also provide a MoveIt-ready shim that converts planned trajectories into streamed MotorCmd for sim (and later, real Z1).



