#!/usr/bin/env python3
import rospy
import time
import datetime
import copy
import sys
import os
from pymycobot import MyArmM, MyArmC

# 저장할 폴더 경로 설정
SAVE_DIR = "/home/aloha/catkin_ws/src/mc_embodied_kit_ros/myarm_m/scripts/recorded_data"
os.makedirs(SAVE_DIR, exist_ok=True)

def get_save_path():
    return os.path.join(SAVE_DIR, "record.txt")

def save_angles_to_file(angles):
    with open(get_save_path(), "a") as f:
        f.write(",".join(map(str, angles)) + "\n")

def load_angles_from_file():
    with open(get_save_path(), "r") as f:
        lines = f.readlines()
    return [list(map(float, line.strip().split(","))) for line in lines]

def record_motion(myarm, duration=10, interval=0.1):
    """ 기록 모드: 지정된 시간 동안 로봇의 관절 각도를 저장 """
    print("Waiting for 5 seconds before recording...")
    time.sleep(5)
    print("Recording motion...")
    start_time = time.time()
    while time.time() - start_time < duration:
        angles = myarm.get_joints_angle()
        if angles:
            save_angles_to_file(angles)
            print("Recorded:", angles)
        time.sleep(interval)
    print("Recording completed!")

def clamp_angle(angle, min_val=-83, max_val=83):
    return max(min(angle, max_val), min_val)

def replay_motion(myarm, speed=30):
    """ 재생 모드: 저장된 각도 데이터를 불러와 로봇을 움직임 """
    print("Replaying motion...")
    recorded_angles = load_angles_from_file()
    
    for angles in recorded_angles:
        clamped_angles = [clamp_angle(a) for a in angles]  # 각도 값 범위 조정
        myarm.set_joints_angle(clamped_angles, speed)
        print("Replaying:", clamped_angles)
        time.sleep(0.1)
    
    print("Replay completed!")


def main():
    if len(sys.argv) < 2:
        print("Usage: python script.py [record/replay]")
        return
    
    mode = sys.argv[1]
    
    if mode == "record":
        myarm = MyArmC('/dev/ttyC1', 1000000, debug=False)  # 기록용 MyArmC
        record_motion(myarm, duration=10, interval=0.1)
    elif mode == "replay":
        myarm = MyArmM('/dev/ttyM1', 1000000, debug=False)  # 재생용 MyArmM
        replay_motion(myarm, speed=30)
    else:
        print("Invalid mode! Use 'record' or 'replay'.")

if __name__ == "__main__":
    main()

