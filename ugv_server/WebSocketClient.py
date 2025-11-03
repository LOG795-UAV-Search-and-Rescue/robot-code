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

    @property
    def steering(self):
        return self._steering

    @steering.setter
    def steering(self, value):
        self._steering = value
        self.update()

    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, value):
        self._throttle = value
        self.update()

    def update(self):
        x = self._throttle * 0.75
        z = self._steering * math.pi * 2
        data = {"T":13,"X":x,"Z":z}
        # print("Sending control data:", data)
        self.controller.base_json_ctrl(data)

    def frame_process(self):
        return self.cvf.raw_frame()


async def send_image(websocket, robot):
    while True:
        ret, frame = robot.frame_process()
        if not ret:
            #print("[send_image] unable to process frame")
            continue
        
        await asyncio.sleep(0.033)  # Approx 30 FPS
        await websocket.send(frame)

async def receive_commands(websocket, robot):
    while True:
        # print("Waiting for command...")
        message = json.loads(await websocket.recv())
        print("Received message:", message)
        # json_car = message.get('Car', {})
        robot.steering = message.get('steering', 0.0)
        robot.throttle = message.get('throttle', 0.0)

async def handle():
    robot = Robot('/dev/ttyTHS1', 115200)
    robot.steering = 0
    robot.throttle = 0
    print("ready to go!")
    # async with websockets.connect("ws://192.168.0.39:5000/robot") as websocket:
    async with websockets.connect("wss://home.adammihajlovic.ca/robot") as websocket:
        await asyncio.gather(
            receive_commands(websocket, robot),
            send_image(websocket, robot),
        )
    await websocket.close()


if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(handle())
