#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import json
import asyncio
import time
import websockets
from base_ctrl import BaseController
import cv_ctrl
import yaml
import math

curpath = os.path.realpath(__file__)
thisPath = os.path.dirname(curpath)
with open(thisPath + '/config.yaml', 'r') as yaml_file:
    f = yaml.safe_load(yaml_file)

class Robot:
    def __init__(self, uart_dev_set, buad_set):
        self.controller = BaseController(uart_dev_set, buad_set)
        self.cvf = cv_ctrl.OpencvFuncs(thisPath, self.controller)
        self._steering = 0.0
        self._throttle = 0.0
        self._head_pan = 0.0
        self._head_tilt = 0.0
        self._head_move = False

    @property
    def steering(self):
        return self._steering

    @steering.setter
    def steering(self, value):
        self._steering = value

    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, value):
        self._throttle = value

    @property
    def head_pan(self):
        return self._head_pan

    @head_pan.setter
    def head_pan(self, value):
        self._head_pan = value

    @property
    def head_tilt(self):
        return self._head_tilt

    @head_tilt.setter
    def head_tilt(self, value):
        self._head_tilt = -value

    @property
    def head_move(self):
        return self._head_move

    @head_move.setter
    def head_move(self, value: bool):
        if value != self._head_move:
            data_head = {"T":133,"X":0,"Y":0,"SPD":0,"ACC":0}
            self.controller.base_json_ctrl(data_head)
        self._head_move = value

    def update(self):
        if(self._head_move):
            data_head = {"T":133,"X":self._head_pan,"Y":self._head_tilt,"SPD":0,"ACC":0}
            # print("Sending head data:", data_head)
            self.controller.base_json_ctrl(data_head)

        x = self._throttle * 1.3
        z = self._steering * math.pi * 2
        data = {"T":13,"X":x,"Z":z}
        # print("Sending control data:", data)
        self.controller.base_json_ctrl(data)    

    def frame_process(self):
        return self.cvf.raw_frame()


async def send_image(websocket, robot):
    while True:
        
        await asyncio.sleep(0.033)  # Approx 30 FPS
        ret, frame = robot.frame_process()
        if not ret:
            # print("[send_image] unable to process frame")
            continue
        
        await websocket.send(frame)

async def receive_commands(websocket, robot):
    while True:
        message = json.loads(await websocket.recv())
        # print("Received message:", message)
        robot.steering = message.get('steering', 0.0)
        robot.throttle = message.get('throttle', 0.0)
        robot.head_pan = message.get('headPanDeg', 0.0)
        robot.head_tilt = message.get('headTiltDeg', 0.0)
        robot.head_move = message.get('headMove', False)
        robot.update()

async def handle():
    robot = Robot('/dev/ttyTHS1', 115200)
    robot.steering = 0
    robot.throttle = 0
    print("ready to go!")
    # async with websockets.connect("ws://192.168.0.39:5000/robot") as websocket:
    while True:
        # async with websockets.connect("ws://192.168.0.39:5000/robot") as websocket:
        async with websockets.connect("wss://home.adammihajlovic.ca/robot") as websocket:
            await asyncio.gather(
                receive_commands(websocket, robot),
                send_image(websocket, robot),
            )
        await websocket.close()


if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(handle())
