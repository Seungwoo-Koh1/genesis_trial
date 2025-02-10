#!/usr/bin/env python3
# encoding:utf-8
from pymycobot import MyArmM, MyArmC
from sensor_msgs.msg import JointState
import rospy
from std_msgs.msg import Header
from math import pi
import subprocess
import sys
import time
import datetime
import copy
import traceback
import os

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

def clamp_angle(angle, min_val=-83, max_val=83):
    return max(min(angle, max_val), min_val)

def wait_for_robot(myarm):
    """ 로봇이 응답할 때까지 대기 """
    print("Waiting for robot connection...")
    while True:
        try:
            angles = myarm.get_joints_angle()
            if angles:
                print("Robot is ready!")
                break
        except Exception as e:
            print(f"Waiting for robot... {e}")
        time.sleep(1)

def record_teleop_motion(myarm, duration=10, interval=0.1):
    """ Teleoperation 수행 중 기록 """
    print("Starting teleoperation...")
    teleop_process = subprocess.Popen(["python3", "/home/aloha/catkin_ws/src/mc_embodied_kit_ros/myarm_m/scripts/mc_embodied_control.py"])
    time.sleep(2)  # Teleoperation 안정화를 위해 약간의 대기 시간 추가
    print("Recording teleoperation motion...")
    start_time = time.time()
    while time.time() - start_time < duration:
        angles = myarm.get_joints_angle()
        if angles:
            save_angles_to_file(angles)
            print("Recorded:", angles)
        time.sleep(interval)
    print("Recording completed! Stopping teleoperation...")
    teleop_process.terminate()
    teleop_process.wait()
    print("Teleoperation process terminated.")

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
    
    myarm = MyArmM('/dev/ttyM1', 1000000, debug=False)  # Teleoperation 및 기록용 MyArmM
    wait_for_robot(myarm)  # 로봇 응답 대기
    
    if mode == "record":
        record_teleop_motion(myarm, duration=10, interval=0.1)
    elif mode == "replay":
        replay_motion(myarm, speed=30)
    else:
        print("Invalid mode! Use 'record' or 'replay'.")

if __name__ == "__main__":
    main()

