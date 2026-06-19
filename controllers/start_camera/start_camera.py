#!/usr/bin/env python3
# -*-coding:utf-8 -*-
"""1.6camera_scene_controller.py
Time    :   2025/06/04
Author  :   ryan 
Version :   1.0
Contact :   ryan@163.com
License :   (C)Copyright 2022, robottime / robodyno

Summary

  视觉小场景夹持器搬运-摄像头控制器
"""
import cv2
import numpy as np
from controller import Robot
from multiprocessing.connection import Client, Listener

webots_robot = Robot()

robot_conn = Client(('localhost', 6001))

cv2.startWindowThread()
cv2.namedWindow("preview")

class RobotCamera(object):
    def __init__(self):
        # 面积设定阈值
        self.min_area = 1000
        self.max_area = 8000
        self.max_object_side = 100
        # 红色HSV阈值
        self.red_lower_hsv = np.array([0, 43, 46])
        self.red_upper_hsv = np.array([10, 255, 255])
        # 蓝色HSV阈值
        self.blue_lower_hsv = np.array([105, 80, 80])
        self.blue_upper_hsv = np.array([125, 255, 255])
        # 绿色HSV阈值
        self.green_lower_hsv = np.array([35, 43, 46])
        self.green_upper_hsv = np.array([77, 255, 255])
        # 定义仿真摄像头
        self.time_step = int(webots_robot.getBasicTimeStep())
        self.camera = webots_robot.getDevice("camera")
        self.camera.enable(self.time_step)

    def find_objs_by_color(self, img, lower_hsv, upper_hsv):
        img = img.copy()
        results = []

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        sharpen = cv2.filter2D(mask, -1, sharpen_kernel)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        close = cv2.morphologyEx(sharpen, cv2.MORPH_CLOSE, kernel, iterations=2)

        cnts = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        for c in cnts:
            area = cv2.contourArea(c)
            if self.min_area < area < self.max_area:
                rect = cv2.minAreaRect(c)
                side_w, side_h = rect[1]

                if side_w > self.max_object_side or side_h > self.max_object_side:
                    continue

                box = np.intp(cv2.boxPoints(rect))

                center_x, center_y = int(rect[0][0]), int(rect[0][1])
                yaw = round(np.pi / 2 - np.radians(rect[2]), 2)

                cv2.polylines(img, [box], isClosed=True, color=(0,255,255), thickness=2)
                cv2.circle(img, (center_x, center_y), radius=3, color=(0,255,255), thickness=-1)
                cv2.putText(
                    img,
                    f"({center_x}, {center_y}, {yaw})",
                    (center_x, center_y - 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0,255,255),
                    1
                )

                results.append((center_x, center_y, yaw))

        return img, results

robot_camera = RobotCamera()

while webots_robot.step(robot_camera.time_step)!= -1: 
    # 读取图像帧
    cameraData = robot_camera.camera.getImage()
    image = np.frombuffer(cameraData, np.uint8).reshape((robot_camera.camera.getHeight(), robot_camera.camera.getWidth(), 4))
    # 画面裁剪
    # image = image[0:480, 200:640]
    all_results = []
    #检测红色物体
    image, red_results = robot_camera.find_objs_by_color(
        image,
        robot_camera.red_lower_hsv,
        robot_camera.red_upper_hsv
    )

    for eye_x, eye_y, yaw in red_results:
        all_results.append({
            "color": "r",
            "x": eye_x,
            "y": eye_y,
            "yaw": yaw
        })
    #检测绿色物体
    image, green_results = robot_camera.find_objs_by_color(
        image,
        robot_camera.green_lower_hsv,
        robot_camera.green_upper_hsv
    )

    for eye_x, eye_y, yaw in green_results:
        all_results.append({
            "color": "g",
            "x": eye_x,
            "y": eye_y,
            "yaw": yaw
        })
    #检测蓝色物体
    image, blue_results = robot_camera.find_objs_by_color(
        image,
        robot_camera.blue_lower_hsv,
        robot_camera.blue_upper_hsv
    )

    for eye_x, eye_y, yaw in blue_results:
        all_results.append({
            "color": "b",
            "x": eye_x,
            "y": eye_y,
            "yaw": yaw
        })

    if all_results:
        robot_conn.send(all_results)
        
    cv2.imshow("preview", image)
    cv2.waitKey(robot_camera.time_step)  

# 关闭连接
# robot_conn.close()
