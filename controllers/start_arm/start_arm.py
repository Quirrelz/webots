#!/usr/bin/env python3
# -*-coding:utf-8 -*-
"""1.6six_dof_robot_gripper_scene_handling.py
Time    :   2025/06/04
Author  :   ryan 
Version :   1.0
Contact :   ryan@163.com
License :   (C)Copyright 2022, robottime / robodyno

Summary

  六自由度双指夹持器机械臂控制器--夹持器搬运
"""
from controller import Robot
from math import pi
import json
from robodyno.components import Motor
from robodyno.interfaces import Webots
from multiprocessing.connection import Listener, Client
from robodyno.robots.six_dof_collaborative_robot import SixDoFCollabRobot

robot_listener = Listener(('localhost', 6001))
robot_accept = robot_listener.accept()
    
webots = Webots()

ROBOT_ID = "start_arm"
MANAGER_ID = "msg_manager"

arm_emitter = webots.robot.getDevice("start_arm_emitter")

arm_receiver = webots.robot.getDevice("start_arm_receiver")
arm_receiver.enable(webots.time_step)

webots.sleep(1)

class MySixDoFArm(SixDoFCollabRobot):
    def __init__(self):
        M1 = Motor(webots, 0x10)
        M2 = Motor(webots, 0x11)
        M3 = Motor(webots, 0x12)
        M4 = Motor(webots, 0x13)
        M5 = Motor(webots, 0x14)
        M6 = Motor(webots, 0x15)
        V1 = webots.robot.getDevice("0x21") 
        # 末端到工具中心点的距离(Tool Center Point) 
        tcp_length = 0.045 
        super().__init__(M1, M2, M3, M4, M5, M6, 0.065, 0.150, 0.150, 0.08, 0.075, 0.045+tcp_length, V1)
    
    def pick(self, x, y, z=0.05, roll=-pi, pitch=0, yaw=-pi/2):
        self.joint_space_interpolated_motion(
            self.inverse_kinematics(x, y, z + 0.1, roll, pitch, yaw), duration=3)
        webots.sleep(1)

        self.joint_space_interpolated_motion(
            self.inverse_kinematics(x, y, z, roll, pitch, yaw), duration=2)
        webots.sleep(1)

        self.end_effector.turnOn()
        webots.sleep(1)

        self.joint_space_interpolated_motion(
            self.inverse_kinematics(x, y, z + 0.1, roll, pitch, yaw), duration=3)
        webots.sleep(1)
    
    def place(self, x, y, z, roll=-pi, pitch=0, yaw=-pi/2):
        self.joint_space_interpolated_motion(
            self.inverse_kinematics(x, y, z + 0.1, roll, pitch, yaw), duration=3)
        webots.sleep(1)

        self.joint_space_interpolated_motion(
            self.inverse_kinematics(x, y, z, roll, pitch, yaw), duration=2)
        webots.sleep(1)

        self.end_effector.turnOff()
        webots.sleep(1)

        self.joint_space_interpolated_motion(
            self.inverse_kinematics(x, y, z + 0.1, roll, pitch, yaw), duration=3)
        webots.sleep(1)
    
    def hand_eye_calib(self, eye_x, eye_y):        
        """手眼标定
        
        参数:
            eye_x: 摄像头的 x 值(pixel)
            eye_y: 摄像头的 y 值(pixel)
        
        返回值:
            hand_x: 机械臂的 x 值(m)
            hand_y: 机械臂的 y 值(m)
        """
        hand_x = 0.0007 * eye_y - 0.0626
        hand_y = 0.0007 * eye_x - 0.0913
        return (hand_x, hand_y)
    
    def get_object_pose(self, target_color):
        latest_msg = None

        while robot_accept.poll():
            latest_msg = robot_accept.recv()

        if not latest_msg:
            return (None, None, None, None)

        target_objs = [
            obj for obj in latest_msg
            if obj.get("color") == target_color
        ]

        if not target_objs:
            return (None, None, None, None)

        # 先抓图像中更靠下的目标
        target = max(target_objs, key=lambda obj: obj["y"])

        hand_x, hand_y = self.hand_eye_calib(float(target["x"]), float(target["y"]))

        return (target_color, hand_x, hand_y, float(target["yaw"]))


    def pick_all_objects_by_color(self, target_color):
        if target_color not in ("r", "g", "b"):
            raise ValueError("target_color must be one of: 'r', 'g', 'b'")

        picked_count = 0
        miss_count = 0
        max_miss_count = 80

        while webots.sleep(0.032) != -1:
            color, hand_x, hand_y, yaw = self.get_object_pose(target_color)

            if color == target_color:
                miss_count = 0

                print(f"Pick {target_color} object {picked_count + 1}")
                self.pick(hand_x, hand_y, yaw=yaw)

                # 放置逻辑保持不变
                self.place(0.15, -0.25-0.06 * picked_count, 0.08 )

                picked_count += 1
            else:
                miss_count += 1

                if miss_count >= max_miss_count:
                    print(f"No more {target_color} objects detected")
                    break

        return picked_count

def send_packet(target, command, data=None):
    packet = {
        "from": ROBOT_ID,
        "to": target,
        "command": command,
        "data": data or {},
        "time": webots.time(),
    }

    arm_emitter.send(json.dumps(packet).encode("utf-8"))
    print("Arm SEND:", packet)


def read_packets():
    packets = []

    while arm_receiver.getQueueLength() > 0:
        raw = arm_receiver.getString()

        try:
            packet = json.loads(raw)
        except json.JSONDecodeError:
            print("Arm received invalid packet:", raw)
            arm_receiver.nextPacket()
            continue

        if packet.get("to") not in (ROBOT_ID, "broadcast"):
            arm_receiver.nextPacket()
            continue

        if packet.get("from") == ROBOT_ID:
            arm_receiver.nextPacket()
            continue

        packets.append(packet)

        arm_receiver.nextPacket()

    return packets

arm = MySixDoFArm()
print('six dof arm inited')

# 初始化机械臂
arm.init() 
# 机械臂使能
arm.enable()
# 末端执行器使能
arm.end_effector.enablePresence(webots.time_step)


# 回零点
print("Arm waiting for msg_manager command...")



while webots.sleep(0.032) != -1:
    for packet in read_packets():
        if packet.get("from") != MANAGER_ID:
            continue

        command = packet.get("command")
        data = packet.get("data", {})

        print("Arm received:", packet)

        if command == "pick_place_color":
            color = data.get("color", "r")

            try:
                
                picked_count = arm.pick_all_objects_by_color(color)
                print("Arm picked count:", picked_count)

                arm.home(2)
                arm.end_effector.turnOff()

                send_packet(
                    MANAGER_ID,
                    "arm_done",
                    {
                        "color": color,
                        "picked_count": picked_count,
                    },
                )

            except Exception as e:
                print("Arm command failed:", e)

                send_packet(
                    MANAGER_ID,
                    "arm_failed",
                    {
                        "command": command,
                        "error": str(e),
                    },
                )

# 关闭连接
# robot_accept.close()
