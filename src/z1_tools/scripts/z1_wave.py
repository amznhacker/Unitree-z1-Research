#!/usr/bin/env python3
import rospy, time, math
from unitree_legged_msgs.msg import MotorCmd

# ---- tune these if needed ----
MODE  = 10         # try 10; if no motion, try 5
KP    = 35.0       # stiffness
KD    = 1.5        # damping
RATE  = 50.0       # Hz
# target "hello" pose (radians) — tweak to match your model
HELLO = {
    "Joint01": 0.0,   # base yaw
    "Joint02": 0.5,   # shoulder pitch up
    "Joint03": 0.8,   # elbow bend
    "Joint04": 0.0,   # forearm roll
    "Joint05": 0.0,   # wrist pitch
    "Joint06": 0.0,   # wrist roll (we'll wave with this)
}
# wave parameters
WAVE_JOINT    = "Joint06"   # wrist roll; try Joint04 if you prefer
WAVE_AMPL     = 0.5         # +/- radians
WAVE_FREQ_HZ  = 1.0         # waves per second
WAVE_SECONDS  = 6.0
# ------------------------------

def pub_for(joint):
    topic = f"/z1_gazebo/{joint}_controller/command"
    return rospy.Publisher(topic, MotorCmd, queue_size=10)

def send(pub, q):
    msg = MotorCmd()
    msg.mode = MODE
    msg.q    = float(q)
    msg.dq   = 0.0
    msg.tau  = 0.0
    msg.Kp   = KP
    msg.Kd   = KD
    pub.publish(msg)

def main():
    rospy.init_node("z1_wave")
    rate = rospy.Rate(RATE)

    joints = ["Joint01","Joint02","Joint03","Joint04","Joint05","Joint06"]
    pubs = {j: pub_for(j) for j in joints}

    # 1) go to hello pose smoothly
    ramp_time = 2.0
    t0 = time.time()
    while not rospy.is_shutdown():
        t = time.time() - t0
        a = min(1.0, t / ramp_time)
        for j in joints:
            q = a * HELLO.get(j, 0.0)
            send(pubs[j], q)
        rate.sleep()
        if a >= 1.0:
            break

    # 2) wave with wrist joint
    t0 = time.time()
    while not rospy.is_shutdown():
        t = time.time() - t0
        if t > WAVE_SECONDS:
            break
        # keep the base pose on other joints
        for j in joints:
            q = HELLO.get(j, 0.0)
            if j == WAVE_JOINT:
                q += WAVE_AMPL * math.sin(2*math.pi*WAVE_FREQ_HZ*t)
            send(pubs[j], q)
        rate.sleep()

    # 3) gently return to neutral
    ramp_back = 1.5
    t0 = time.time()
    while not rospy.is_shutdown():
        t = time.time() - t0
        a = max(0.0, 1.0 - t / ramp_back)
        for j in joints:
            q = a * HELLO.get(j, 0.0)
            send(pubs[j], q)
        rate.sleep()
        if a <= 0.0:
            break

if __name__ == "__main__":
    main()
