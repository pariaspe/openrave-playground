#!/usr/bin/env python


from openravepy import *
import time
import numpy as np
from math import sqrt, atan2

__author__ = "Pedro Arias Perez"


GOAL = [8, 8]
RATE = 1


env = Environment() # create openrave environment
env.SetViewer('qtosg') # attach viewer (optional)

try:
    env.Load('/home/yo/repos/openrave-playground/tools/map.env.xml') # load a simple scene
    robot = env.GetRobots()[0] # get the first robot
    print("Using robot: ", robot.GetName())
    robot.SetController(RaveCreateController(env,'idealvelocitycontroller'),range(robot.GetDOF()),0)
    control = robot.GetController()

    env.StartSimulation(timestep=0.001)

    time.sleep(2)  # wait to viewer

    # THE ROBOT WILL GO [1.6, 8], TURN TO THE GOAL AND THEN MOVE FORWARD

    print("\nGOING TO POINT 1 [1.6, 8]")
    while True:
        x, y, _ = np.round(robot.GetTransform()[:3, -1,].T, 2)  # getting pose from ecro

        dx = 1.6 - x
        dy = 8.0 - y
        d = sqrt(dx**2 + dy**2)  # distance to point
        control.SetDesired([d, d, d, d])  # set vel
        print(d)
        if abs(d) <= 0.1:
            break  # point reached
        time.sleep(1/RATE)  # spin

    print("\nTURNING")
    while True:
        x, y, _ = np.round(robot.GetTransform()[:3, -1,].T, 2)  # getting pose from ecro
        dx = GOAL[0] - x
        dy = GOAL[1] - y

        goal_heading = atan2(dy, dx)  # heading tp goal
        current_heading = np.round(axisAngleFromRotationMatrix(robot.GetTransform()[:3, :3]), 3)[-1]  # getting heading from ecro
        vel = current_heading - goal_heading
        vel *= 2  # double vel
        print(vel)
        control.SetDesired([vel, -vel, vel, -vel])  # turning
        if abs(vel) <= 0.01:
            break  # heading reached
        time.sleep(1/RATE)  # spin

    print("\nGOING TO POINT GOAL")
    while True:
        x, y, _ = np.round(robot.GetTransform()[:3, -1,].T, 2)  # getting pose from ecro

        dx = GOAL[0] - x
        dy = GOAL[1] - y
        d = sqrt(dx**2 + dy**2)  # distance to point
        control.SetDesired([d, d, d, d])  # set vel
        print(d)
        if abs(d) <= 0.1:
            break  # point reached
        time.sleep(1/RATE)  # spin

    print("\nGOAL REACHED!!\n")
    print("BYE!")

    time.sleep(1)

finally:
    env.Destroy()
