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
