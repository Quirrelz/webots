#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
from controller import Robot
from multiprocessing.connection import Client


CAMERA_PORT = 6003

webots_robot = Robot()
robot_conn = Client(("localhost", CAMERA_PORT))

cv2.startWindowThread()
cv2.namedWindow("preview")


class RobotCamera:
    def __init__(self):
        self.min_area = 1000
        self.max_area = 8000
        self.max_object_side = 100

        self.red_lower_hsv = np.array([0, 43, 46])
        self.red_upper_hsv = np.array([10, 255, 255])
        self.green_lower_hsv = np.array([35, 43, 46])
        self.green_upper_hsv = np.array([77, 255, 255])
        self.blue_lower_hsv = np.array([105, 80, 80])
        self.blue_upper_hsv = np.array([120, 255, 255])

        self.time_step = int(webots_robot.getBasicTimeStep())
        self.camera = webots_robot.getDevice("camera")
        self.camera.enable(self.time_step)

    def find_objs_by_color(self, img, lower_hsv, upper_hsv, draw_color):
        results = []
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        sharpen_kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
        sharpen = cv2.filter2D(mask, -1, sharpen_kernel)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        close = cv2.morphologyEx(sharpen, cv2.MORPH_CLOSE, kernel, iterations=2)

        cnts = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        for contour in cnts:
            area = cv2.contourArea(contour)
            if self.min_area < area < self.max_area:
                x, y, w, h = cv2.boundingRect(contour)

                if w > self.max_object_side or h > self.max_object_side:
                    continue

                center_x = x + w / 2
                center_y = y + h / 2
                results.append((center_x, center_y))

                cv2.putText(
                    img,
                    "center: ({:.1f}, {:.1f})".format(center_x, center_y),
                    (x, y - 40),
                    cv2.FONT_HERSHEY_COMPLEX,
                    0.5,
                    draw_color,
                    1,
                )
                cv2.rectangle(img, (x, y), (x + w, y + h), draw_color, 2)

        return results


robot_camera = RobotCamera()

while webots_robot.step(robot_camera.time_step) != -1:
    camera_data = robot_camera.camera.getImage()
    image = np.frombuffer(camera_data, np.uint8).reshape(
        (robot_camera.camera.getHeight(), robot_camera.camera.getWidth(), 4)
    )
    image = image[0:300, 0:640].copy()

    color_specs = [
        ("r", robot_camera.red_lower_hsv, robot_camera.red_upper_hsv, (0, 0, 255)),
        ("g", robot_camera.green_lower_hsv, robot_camera.green_upper_hsv, (0, 255, 0)),
        ("b", robot_camera.blue_lower_hsv, robot_camera.blue_upper_hsv, (255, 0, 0)),
    ]

    for color, lower_hsv, upper_hsv, draw_color in color_specs:
        for eye_x, eye_y in robot_camera.find_objs_by_color(image, lower_hsv, upper_hsv, draw_color):
            robot_conn.send("{}, {}, {}".format(color, eye_x, eye_y))

    cv2.imshow("preview", image)
    cv2.waitKey(robot_camera.time_step)
