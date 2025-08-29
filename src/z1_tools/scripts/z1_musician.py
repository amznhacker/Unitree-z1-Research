#!/usr/bin/env python3
"""
Z1 Musician
Play air instruments and conduct orchestras
"""
import rospy, math, time, argparse
from unitree_legged_msgs.msg import MotorCmd

class Z1Musician:
    def __init__(self, mode=10, kp=35.0, kd=1.5):
        rospy.init_node("z1_musician")
        
        self.mode = mode
        self.kp = kp
        self.kd = kd
        
        # Publishers
        self.pubs = {}
        joints = ["Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06", "Gripper"]
        for joint in joints:
            topic = f"/z1_gazebo/{joint}_controller/command"
            self.pubs[joint] = rospy.Publisher(topic, MotorCmd, queue_size=10)
        
        self.rate = rospy.Rate(50)
        
        # Musical poses
        self.poses = {
            "conductor_ready": [0.0, -0.2, 0.6, 0.0, -0.4, 0.0, 0.5],
            "violin_hold": [0.5, -0.1, 0.8, -0.3, -0.7, 0.2, 0.3],
            "piano_left": [-0.4, -0.3, 0.9, 0.0, -0.6, 0.0, 0.8],
            "piano_right": [0.4, -0.3, 0.9, 0.0, -0.6, 0.0, 0.8],
            "drums_ready": [0.0, 0.1, 0.4, 0.0, 0.3, 0.0, 0.2],
            "guitar_strum": [0.3, -0.2, 0.7, 0.0, -0.5, 0.0, 0.8]
        }
        
        # Musical scales and rhythms
        self.scales = {
            "c_major": [0, 2, 4, 5, 7, 9, 11, 12],  # Semitone intervals
            "pentatonic": [0, 2, 4, 7, 9, 12],
            "blues": [0, 3, 5, 6, 7, 10, 12]
        }
    
    def send_pose(self, pose_name_or_positions, duration=2.0):
        """Move to pose smoothly"""
        if isinstance(pose_name_or_positions, str):
            if pose_name_or_positions in self.poses:
                target = self.poses[pose_name_or_positions]
            else:
                rospy.logerr(f"Unknown pose: {pose_name_or_positions}")
                return
        else:
            target = pose_name_or_positions
        
        start_time = time.time()
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration:
                break
            
            alpha = elapsed / duration
            smooth_alpha = 0.5 * (1 - math.cos(math.pi * alpha))
            
            joints = ["Joint01", "Joint02", "Joint03", "Joint04", "Joint05", "Joint06", "Gripper"]
            for i, joint in enumerate(joints):
                if i < len(target):
                    position = target[i] * smooth_alpha
                    
                    msg = MotorCmd()
                    msg.mode = self.mode
                    msg.q = float(position)
                    msg.dq = 0.0
                    msg.tau = 0.0
                    msg.Kp = self.kp
                    msg.Kd = self.kd
                    self.pubs[joint].publish(msg)
            
            self.rate.sleep()
    
    def conduct_orchestra(self, tempo_bpm=120, measures=8):
        """Conduct an orchestra with expressive gestures"""
        rospy.loginfo(f"🎼 Conducting orchestra at {tempo_bpm} BPM for {measures} measures")
        
        # Move to conductor position
        self.send_pose("conductor_ready", 2.0)
        
        beat_duration = 60.0 / tempo_bpm  # seconds per beat
        
        for measure in range(measures):
            rospy.loginfo(f"Measure {measure + 1}/{measures}")
            
            for beat in range(4):  # 4/4 time
                # Downbeat emphasis
                if beat == 0:
                    # Strong downbeat
                    downbeat = [0.0, 0.3, 0.2, 0.0, 0.5, 0.0, 0.5]
                    self.send_pose(downbeat, beat_duration * 0.3)
                    
                    # Follow through
                    follow = [0.0, -0.4, 0.8, 0.0, -0.6, 0.0, 0.5]
                    self.send_pose(follow, beat_duration * 0.7)
                else:
                    # Regular beats - flowing motion
                    beat_height = 0.1 + beat * 0.05
                    beat_pose = [0.0, -0.2 + beat_height, 0.6, 0.0, -0.4, 0.0, 0.5]
                    self.send_pose(beat_pose, beat_duration * 0.5)
                    
                    # Return to ready
                    self.send_pose("conductor_ready", beat_duration * 0.5)
        
        # Grand finale gesture
        finale = [0.0, 0.8, -0.2, 0.0, 0.6, 0.0, 0.8]
        self.send_pose(finale, 2.0)
        rospy.loginfo("🎉 Magnificent performance!")
    
    def play_air_violin(self, melody_length=16):
        """Play air violin with bow movements"""
        rospy.loginfo(f"🎻 Playing air violin - {melody_length} notes")
        
        # Hold violin position
        self.send_pose("violin_hold", 2.0)
        
        base_pose = self.poses["violin_hold"]
        
        for note in range(melody_length):
            # Bow movement across strings
            bow_phase = (note % 4) / 4.0  # 4 notes per bow direction
            
            # Alternate bow direction
            if (note // 4) % 2 == 0:
                bow_pos = bow_phase  # Down bow
            else:
                bow_pos = 1.0 - bow_phase  # Up bow
            
            # Map to wrist movement
            wrist_pos = -0.3 + bow_pos * 0.6
            
            # Add vibrato with forearm
            vibrato = 0.05 * math.sin(note * 2)
            
            violin_pose = base_pose.copy()
            violin_pose[4] = wrist_pos  # Bow movement
            violin_pose[3] += vibrato   # Vibrato
            
            # Note duration
            note_duration = 0.5
            self.send_pose(violin_pose, note_duration)
        
        rospy.loginfo("🎵 Beautiful violin melody complete!")
    
    def play_air_piano(self, scale="c_major", octaves=2):
        """Play air piano with both hands"""
        rospy.loginfo(f"🎹 Playing air piano - {scale} scale, {octaves} octaves")
        
        scale_notes = self.scales.get(scale, self.scales["c_major"])
        
        for octave in range(octaves):
            for i, note in enumerate(scale_notes):
                # Alternate hands
                if i % 2 == 0:
                    # Left hand
                    self.send_pose("piano_left", 0.3)
                    # Key press
                    press_pose = self.poses["piano_left"].copy()
                    press_pose[1] -= 0.1  # Press down
                    self.send_pose(press_pose, 0.1)
                    # Release
                    self.send_pose("piano_left", 0.2)
                else:
                    # Right hand
                    self.send_pose("piano_right", 0.3)
                    # Key press
                    press_pose = self.poses["piano_right"].copy()
                    press_pose[1] -= 0.1  # Press down
                    self.send_pose(press_pose, 0.1)
                    # Release
                    self.send_pose("piano_right", 0.2)
        
        # Final chord - both hands
        rospy.loginfo("🎼 Grand finale chord!")
        both_hands_down = [0.0, -0.4, 1.0, 0.0, -0.7, 0.0, 0.8]
        self.send_pose(both_hands_down, 0.5)
        
        # Lift hands dramatically
        both_hands_up = [0.0, 0.2, 0.3, 0.0, 0.1, 0.0, 0.8]
        self.send_pose(both_hands_up, 1.5)
    
    def play_air_drums(self, pattern_length=16):
        """Play air drums with rhythm"""
        rospy.loginfo(f"🥁 Playing air drums - {pattern_length} beat pattern")
        
        # Ready position
        self.send_pose("drums_ready", 2.0)
        
        base_pose = self.poses["drums_ready"]
        
        for beat in range(pattern_length):
            # Different drum sounds based on position
            if beat % 4 == 0:
                # Kick drum (center, down)
                drum_pose = [0.0, -0.2, 0.8, 0.0, -0.4, 0.0, 0.1]
                rospy.loginfo("🎵 KICK")
            elif beat % 4 == 2:
                # Snare (slight left)
                drum_pose = [-0.2, 0.0, 0.5, 0.0, 0.2, 0.0, 0.1]
                rospy.loginfo("🎵 SNARE")
            elif beat % 2 == 1:
                # Hi-hat (right, higher)
                drum_pose = [0.3, 0.2, 0.3, 0.0, 0.4, 0.0, 0.1]
                rospy.loginfo("🎵 hi-hat")
            else:
                # Tom (center-right)
                drum_pose = [0.1, 0.1, 0.4, 0.0, 0.3, 0.0, 0.1]
                rospy.loginfo("🎵 tom")
            
            # Strike
            self.send_pose(drum_pose, 0.1)
            
            # Return to ready
            self.send_pose("drums_ready", 0.2)
        
        # Final crash cymbal
        rospy.loginfo("💥 CRASH!")
        crash_pose = [0.5, 0.4, 0.1, 0.0, 0.6, 0.0, 0.8]
        self.send_pose(crash_pose, 1.0)
    
    def strum_air_guitar(self, chords=8):
        """Strum air guitar with rock moves"""
        rospy.loginfo(f"🎸 Rocking out on air guitar - {chords} chords")
        
        # Guitar position
        self.send_pose("guitar_strum", 2.0)
        
        base_pose = self.poses["guitar_strum"]
        
        for chord in range(chords):
            # Power chord positions
            if chord % 4 == 0:
                # Low E chord
                fret_pos = base_pose.copy()
                fret_pos[1] -= 0.1
                rospy.loginfo("🎸 E POWER CHORD!")
            elif chord % 4 == 1:
                # A chord
                fret_pos = base_pose.copy()
                fret_pos[1] += 0.05
                rospy.loginfo("🎸 A CHORD!")
            elif chord % 4 == 2:
                # D chord
                fret_pos = base_pose.copy()
                fret_pos[1] += 0.1
                rospy.loginfo("🎸 D CHORD!")
            else:
                # G chord
                fret_pos = base_pose.copy()
                rospy.loginfo("🎸 G CHORD!")
            
            # Strum down
            strum_down = fret_pos.copy()
            strum_down[5] = -0.5  # Wrist down strum
            self.send_pose(strum_down, 0.2)
            
            # Strum up
            strum_up = fret_pos.copy()
            strum_up[5] = 0.5   # Wrist up strum
            self.send_pose(strum_up, 0.3)
            
            # Return to position
            self.send_pose(fret_pos, 0.3)
        
        # Rock star finale
        rospy.loginfo("🤘 ROCK ON!")
        rock_pose = [0.0, 0.5, 0.2, 0.0, 0.3, 0.0, 0.8]
        self.send_pose(rock_pose, 2.0)
    
    def musical_performance(self):
        """Full musical performance showcase"""
        rospy.loginfo("🎭 Welcome to Z1's Musical Showcase!")
        
        performances = [
            ("Conducting Beethoven's 5th", lambda: self.conduct_orchestra(120, 4)),
            ("Violin Concerto", lambda: self.play_air_violin(12)),
            ("Piano Sonata", lambda: self.play_air_piano("c_major", 1)),
            ("Rock Drum Solo", lambda: self.play_air_drums(12)),
            ("Guitar Rock Anthem", lambda: self.strum_air_guitar(6))
        ]
        
        for title, performance in performances:
            rospy.loginfo(f"\\n🎪 Now performing: {title}")
            performance()
            time.sleep(2.0)
        
        # Final bow
        rospy.loginfo("🎉 Thank you! Z1 has left the building!")
        bow_pose = [0.0, -0.8, 1.5, 0.0, -0.7, 0.0, 0.0]
        self.send_pose(bow_pose, 3.0)
    
    def return_home(self):
        """Return to rest position"""
        rospy.loginfo("🎵 Performance complete - returning to green room")
        self.send_pose([0.0] * 7, 3.0)

def main():
    parser = argparse.ArgumentParser(description="Z1 Master Musician")
    parser.add_argument("--mode", type=int, default=10)
    parser.add_argument("--kp", type=float, default=35.0)
    parser.add_argument("--kd", type=float, default=1.5)
    parser.add_argument("--instrument", choices=["conductor", "violin", "piano", "drums", "guitar", "concert"],
                       default="concert", help="Instrument to play or full concert")
    parser.add_argument("--tempo", type=int, default=120, help="Tempo in BPM for conducting")
    parser.add_argument("--scale", choices=["c_major", "pentatonic", "blues"], 
                       default="c_major", help="Scale for piano")
    args = parser.parse_args()
    
    try:
        musician = Z1Musician(args.mode, args.kp, args.kd)
        
        if args.instrument == "conductor":
            musician.conduct_orchestra(args.tempo, 6)
        elif args.instrument == "violin":
            musician.play_air_violin(16)
        elif args.instrument == "piano":
            musician.play_air_piano(args.scale, 2)
        elif args.instrument == "drums":
            musician.play_air_drums(16)
        elif args.instrument == "guitar":
            musician.strum_air_guitar(8)
        else:  # concert
            musician.musical_performance()
        
        musician.return_home()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Concert interrupted")

if __name__ == "__main__":
    main()