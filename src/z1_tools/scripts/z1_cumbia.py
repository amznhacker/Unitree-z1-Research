#!/usr/bin/env python3
import rospy, math, time, argparse
from unitree_legged_msgs.msg import MotorCmd

def pub_for(j):
    return rospy.Publisher(f"/z1_gazebo/{j}_controller/command", MotorCmd, queue_size=10)

def send(pub, q, mode, kp, kd):
    m = MotorCmd()
    m.mode = mode
    m.q    = float(q)
    m.dq   = 0.0
    m.tau  = 0.0
    m.Kp   = kp
    m.Kd   = kd
    pub.publish(m)

def clamp(x, lo, hi): return max(lo, min(hi, x))

def main():
    ap = argparse.ArgumentParser(description="Z1 Cumbia dance (MotorCmd)")
    ap.add_argument("--mode", type=int, default=10, help="Controller mode (try 10, else 5)")
    ap.add_argument("--kp", type=float, default=35.0)
    ap.add_argument("--kd", type=float, default=1.5)
    ap.add_argument("--bpm", type=float, default=96.0, help="Cumbia tempo (beats per minute)")
    ap.add_argument("--dur", type=float, default=20.0, help="Duration seconds")
    # amplitudes (radians)
    ap.add_argument("--amp_base", type=float, default=0.35)     # Joint01 yaw sway
    ap.add_argument("--amp_sh",   type=float, default=0.45)     # Joint02 shoulder (pitch)
    ap.add_argument("--amp_el",   type=float, default=0.70)     # Joint03 elbow
    ap.add_argument("--amp_fr",   type=float, default=0.35)     # Joint04 forearm roll
    ap.add_argument("--amp_wp",   type=float, default=0.30)     # Joint05 wrist pitch
    ap.add_argument("--amp_wr",   type=float, default=0.60)     # Joint06 wrist roll (flourish)
    ap.add_argument("--snap", action="store_true", help="Add gripper snaps on the backbeat")
    args = ap.parse_args()

    rospy.init_node("z1_cumbia")
    rate = rospy.Rate(50)  # 50 Hz
    pubs = {j: pub_for(j) for j in ["Joint01","Joint02","Joint03","Joint04","Joint05","Joint06","Gripper"]}

    # Timing
    beat = 60.0 / args.bpm          # seconds per beat
    step_freq = 1.0/beat            # Hz; drive main groove with beat
    t0 = time.time()

    # Bring to neutral first (1.5 s ramp)
    ramp = 1.5
    while not rospy.is_shutdown():
        t = time.time() - t0
        a = min(1.0, t/ramp)
        for j in ["Joint01","Joint02","Joint03","Joint04","Joint05","Joint06"]:
            send(pubs[j], 0.0*a, args.mode, args.kp, args.kd)
        rate.sleep()
        if a >= 1.0: break

    start = time.time()
    while not rospy.is_shutdown():
        t = time.time() - start
        if t > args.dur: break

        # Phase helpers
        # Cumbia 2-step: accent on beat 2 & 4
        phase = 2*math.pi*step_freq*t
        # Sways & grooves (use phase offsets to look musical)
        q01 = args.amp_base * math.sin(phase)                    # base yaw left/right
        q02 = args.amp_sh   * math.sin(phase + math.pi/2)        # shoulder up on beat
        q03 = args.amp_el   * math.sin(phase + math.pi/2) * 0.7  # elbow follows shoulder
        q04 = args.amp_fr   * math.sin(2*phase) * 0.5            # gentle forearm roll
        q05 = args.amp_wp   * math.sin(phase + math.pi/3) * 0.7  # wrist pitch syncopation
        q06 = args.amp_wr   * math.sin(2*phase + math.pi/2)      # wrist roll flourish (double-time)

        # Clamp to conservative bounds
        q01 = clamp(q01, -1.0, 1.0)
        q02 = clamp(q02, -1.2, 1.2)
        q03 = clamp(q03, -1.4, 1.4)
        q04 = clamp(q04, -1.2, 1.2)
        q05 = clamp(q05, -1.0, 1.0)
        q06 = clamp(q06, -1.5, 1.5)

        send(pubs["Joint01"], q01, args.mode, args.kp, args.kd)
        send(pubs["Joint02"], q02, args.mode, args.kp, args.kd)
        send(pubs["Joint03"], q03, args.mode, args.kp, args.kd)
        send(pubs["Joint04"], q04, args.mode, args.kp, args.kd)
        send(pubs["Joint05"], q05, args.mode, args.kp, args.kd)
        send(pubs["Joint06"], q06, args.mode, args.kp, args.kd)

        # Optional gripper "snaps" on 2 & 4 (square-ish pulse)
        if args.snap:
            beat_idx = int(t/beat)
            # strong on even beats (2 & 4 modulo 4)
            on = (beat_idx % 2 == 1)
            # short pulse (80ms)
            pulse = (t - beat_idx*beat) < 0.08
            qg = 0.25 if (on and pulse) else 0.0
            send(pubs["Gripper"], qg, args.mode, args.kp, args.kd)

        rate.sleep()

    # Smooth exit back to neutral
    t1 = time.time()
    while not rospy.is_shutdown():
        a = max(0.0, 1.0 - (time.time()-t1)/1.0)
        for j in ["Joint01","Joint02","Joint03","Joint04","Joint05","Joint06","Gripper"]:
            send(pubs[j], 0.0*a, args.mode, args.kp, args.kd)
        rate.sleep()
        if a <= 0.0: break

if __name__ == "__main__":
    main()
