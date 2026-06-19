#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from math import pi
import json
from multiprocessing.connection import Listener

from robodyno.components import Motor
from robodyno.interfaces import Webots
from robodyno.robots.six_dof_collaborative_robot import SixDoFCollabRobot


ROBOT_ID = "red_target_arm"
MANAGER_ID = "msg_manager"
CAMERA_PORT = 6002
EMITTER_NAME = "red_target_emitter"
RECEIVER_NAME = "red_target_receiver"


robot_listener = Listener(("localhost", CAMERA_PORT))
robot_accept = robot_listener.accept()

webots = Webots()

emitter = webots.robot.getDevice(EMITTER_NAME)
receiver = webots.robot.getDevice(RECEIVER_NAME)
receiver.enable(webots.time_step)

webots.sleep(1)


class MySixDoFArm(SixDoFCollabRobot):
    def __init__(self):
        motors = [Motor(webots, address) for address in (0x10, 0x11, 0x12, 0x13, 0x14, 0x15)]
        end_effector = webots.robot.getDevice("0x21")
        tcp_length = 0.045
        super().__init__(*motors, 0.065, 0.150, 0.150, 0.08, 0.075, 0.045 + tcp_length, end_effector)

    def pick(self, x, y, z=0.04, roll=-pi, pitch=0, yaw=-pi / 2):
        self.joint_space_interpolated_motion(
            self.inverse_kinematics(x, y, z + 0.1, roll, pitch, yaw), duration=3
        )
        webots.sleep(1)
        self.joint_space_interpolated_motion(
            self.inverse_kinematics(x, y, z, roll, pitch, yaw), duration=1
        )
        webots.sleep(1)
        self.end_effector.turnOn()
        webots.sleep(1)
        self.joint_space_interpolated_motion(
            self.inverse_kinematics(x, y, z + 0.1, roll, pitch, yaw), duration=3
        )
        webots.sleep(1)

    def place(self, x, y, z, roll=-pi, pitch=0, yaw=-pi / 2):
        self.joint_space_interpolated_motion(
            self.inverse_kinematics(x, y, z + 0.1, roll, pitch, yaw), duration=3
        )
        webots.sleep(1)
        self.joint_space_interpolated_motion(
            self.inverse_kinematics(x, y, z, roll, pitch, yaw), duration=1
        )
        webots.sleep(1)
        self.end_effector.turnOff()
        webots.sleep(1)
        self.joint_space_interpolated_motion(
            self.inverse_kinematics(x, y, z + 0.1, roll, pitch, yaw), duration=3
        )
        webots.sleep(1)

    def hand_eye_calib(self, eye_x, eye_y):
        hand_x = 0.0007 * eye_y - 0.0626 - 0.3
        hand_y = 0.0007 * eye_x - 0.0913 
        return hand_x, hand_y

    def get_object_pose(self, target_color=None):
        selected = None

        while robot_accept.poll():
            latest_msg = robot_accept.recv()
            parts = latest_msg.split(",")

            if len(parts) != 3:
                print(ROBOT_ID, "invalid camera message:", latest_msg)
                continue

            color, eye_x, eye_y = parts
            color = color.strip()

            if target_color is None or color == target_color:
                selected = (color, float(eye_x), float(eye_y))

        if selected is not None:
            color, eye_x, eye_y = selected
            hand_x, hand_y = self.hand_eye_calib(eye_x, eye_y)
            return color, hand_x, hand_y

        return None, None, None

    def flush_camera_messages(self):
        flushed = 0

        while robot_accept.poll():
            robot_accept.recv()
            flushed += 1

        if flushed:
            print(ROBOT_ID, "flushed old camera messages:", flushed)

    def pick_place_order(self, picking_order, vision_settle_time=0.3, no_detection_miss_count=80):
        picked = []
        self.flush_camera_messages()

        if vision_settle_time > 0:
            webots.sleep(float(vision_settle_time))

        for pick_color in picking_order:
            miss_count = 0

            while webots.sleep(0.032) != -1:
                color, hand_x, hand_y = self.get_object_pose(pick_color)

                if color == pick_color:
                    miss_count = 0
                    place_index = len(picked)
                    print(ROBOT_ID, "pick", pick_color)
                    self.pick(hand_x, hand_y)
                    self.place(0.20, 0.0, 0.055 + 0.05 * place_index)
                    picked.append(pick_color)
                    self.flush_camera_messages()

                    if vision_settle_time > 0:
                        webots.sleep(float(vision_settle_time))

                    continue

                miss_count += 1

                if miss_count >= int(no_detection_miss_count):
                    print(ROBOT_ID, "no more", pick_color, "objects detected; picked", picked.count(pick_color))
                    break

        self.home(2)
        webots.sleep(1)
        self.end_effector.turnOff()
        return picked


def send_packet(target, command, data=None):
    packet = {
        "from": ROBOT_ID,
        "to": target,
        "command": command,
        "data": data or {},
        "time": webots.time(),
    }
    emitter.send(json.dumps(packet).encode("utf-8"))
    print(ROBOT_ID, "SEND:", packet)


def send_return_command(return_car_id, return_pose, pre_return_straight_distance=0.0):
    if not return_car_id or not return_pose:
        return

    if abs(float(pre_return_straight_distance)) > 1e-6:
        send_packet(
            return_car_id,
            "move_straight",
            {
                "distance": float(pre_return_straight_distance),
                "notify_done": False,
            },
        )

    send_packet(
        return_car_id,
        "go_to",
        {
            "target_x": float(return_pose["x"]),
            "target_y": float(return_pose["y"]),
            "target_heading": float(return_pose["heading"]),
        },
    )


def read_packets():
    packets = []

    while receiver.getQueueLength() > 0:
        raw = receiver.getString()

        try:
            packet = json.loads(raw)
        except json.JSONDecodeError:
            print(ROBOT_ID, "received invalid packet:", raw)
            receiver.nextPacket()
            continue

        receiver.nextPacket()

        if packet.get("to") not in (ROBOT_ID, "broadcast"):
            continue

        if packet.get("from") == ROBOT_ID:
            continue

        packets.append(packet)

    return packets


arm = MySixDoFArm()
print(ROBOT_ID, "inited")

arm.init()
arm.enable()
arm.end_effector.enablePresence(webots.time_step)

print(ROBOT_ID, "waiting for msg_manager command")

while webots.sleep(0.032) != -1:
    for packet in read_packets():
        if packet.get("from") != MANAGER_ID:
            continue

        command = packet.get("command")
        data = packet.get("data", {})

        if command == "manager_ping":
            send_packet(MANAGER_ID, "target_arm_ready")

        elif command in ("pick_place_order", "pick_place_color"):
            try:
                if command == "pick_place_color":
                    order = [data.get("color", "r")]
                else:
                    order = data.get("colors", ["b", "r"])

                picked = arm.pick_place_order(
                    order,
                    float(data.get("vision_settle_time", 0.3)),
                    int(data.get("no_detection_miss_count", 80)),
                )
                return_car_id = data.get("return_car_id")
                return_pose = data.get("return_pose")
                pre_return_straight_distance = float(data.get("pre_return_straight_distance", 0.0))

                send_packet(
                    MANAGER_ID,
                    "target_arm_done",
                    {
                        "command": command,
                        "colors": order,
                        "picked": picked,
                        "return_car_id": return_car_id,
                    },
                )
                send_return_command(return_car_id, return_pose, pre_return_straight_distance)
            except Exception as exc:
                print(ROBOT_ID, "command failed:", exc)
                send_packet(
                    MANAGER_ID,
                    "target_arm_failed",
                    {
                        "command": command,
                        "error": str(exc),
                    },
                )
