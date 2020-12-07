#!/usr/bin/env python

"""
Descripcion
"""

import sys
import os
from openravepy import *
import time
import numpy as np
from math import sqrt, atan2, degrees, radians, pi
from subprocess import Popen, PIPE


__author__ = "Pedro Arias Perez"


LOCAL_PATH = os.path.dirname(os.path.abspath(__file__))
FILE_NAME = LOCAL_PATH + "/tools/assets/{0}.csv"
GOAL = [8, 8]

MAX_VEL = 10
RATE = 1

ANG_TOL = 0.01
LIN_TOL = 0.1


def inflate_square(map, i, j):
    try:
        if map[i][j] != 1:
            map[i][j] = 2
    except:
        pass


def inflate_map(map_name):
    map = []
    with open(FILE_NAME.format(map_name), "r") as f:
        content = f.read()
        for l in content.split("\n"):
            row = []
            for c in l.split(","):
                if c != '':
                    row.append(int(c))
            if row:
                map.append(row)
    f.close()

    for i in range(len(map)):
        for j in range(len(map[0])):
            if map[i][j] == 1:
                inflate_square(map, i-1, j)
                inflate_square(map, i, j-1)
                inflate_square(map, i+1, j)
                inflate_square(map, i, j+1)

    for i in range(len(map)):
        for j in range(len(map[0])):
            if map[i][j] == 2:
                map[i][j] = 1

    with open(FILE_NAME.format(map_name + "_inflated"), "w") as f:
        for i in range(len(map)):
            for j in range(len(map[0])):
                f.write(str(map[i][j]))
                if j != len(map[0]) -1:
                    f.write(",")
            f.write("\n")
    f.close()
    return map_name + "_inflated"


def optimize_route(route):
    redundants = []
    optimized = []

    previous = None
    current = route[0]
    next_ = None
    for i, step in enumerate(route):
        if i == len(route) - 1:
            break
        previous = current
        current = route[i]
        next_ = route[i+1]
        if i == 0:
            continue
        if previous[0] == current[0] == next_[0]:
            redundants.append(current)
        elif previous[1] == current[1] == next_[1]:
            redundants.append(current)

    for step in route:
        if step not in redundants:
            optimized.append(step)

    return optimized

def get_route(map, start, end):
    map_inflated = inflate_map(map)
    process = Popen(["python3", "tools/astar.py", "-m", map_inflated, "-s", str(start[0]), str(start[1]), "-e", str(end[0]), str(end[1])], stdout=PIPE, stderr=PIPE)
    stdout, stderr =  process.communicate()

    output =stdout.split('\n')[:-12]
    i = -1
    while True:
        if output[i] == "%%%%%%%%%%%%%%%%%%%":
            break
        i -= 1
    output = output[i+1:]
    steps = [end]
    for l in output:
        steps.insert(0, [int(l[13]), int(l[19])])
    return optimize_route(steps)


class EcroWrapper:
    def __init__(self, env, ecro):
        self.robot = ecro
        self.robot.SetController(RaveCreateController(env,'idealvelocitycontroller'),range(self.robot.GetDOF()),0)
        self.control = self.robot.GetController()

    def get_pos(self):
        H_0_robot = self.robot.GetTransform()
        return np.round(H_0_robot[:3, -1,].T, 2)

    def get_rot(self):
        H_0_robot = self.robot.GetTransform()
        return H_0_robot[:3, :3]

    def get_angles(self):
        return np.round(axisAngleFromRotationMatrix(self.get_rot()), 3)

    def get_heading(self):
        return self.get_angles()[-1]

    def set_linear_vel(self, vel):
        if vel > MAX_VEL:
            vel = MAX_VEL
        self.control.SetDesired(4*[vel])

    def set_angular_vel(self, vel):
        if vel > MAX_VEL:
            vel = MAX_VEL
        self.control.SetDesired([vel, -vel, vel, -vel])

    def set_cmd_pos(self, pos, block=False):
        if block:
            print("\nHEADING TO ({0}, {1})".format(str(pos[0]), str(pos[1])))
            dif = 1  # init value, just to enter in while
            while abs(dif) > ANG_TOL:
                x, y, _ = self.get_pos()

                dx = pos[0] - x
                dy = pos[1] - y

                goal_heading = atan2(dy, dx)

                dif = self.get_heading() - goal_heading
                self.set_angular_vel(dif*4)

                time.sleep(1/RATE)

            print("GOING TO ({0}, {1})".format(str(pos[0]), str(pos[1])))
            dist = 1
            while abs(dist) > LIN_TOL:
                x, y, _ = self.get_pos()

                dx = pos[0] - x
                dy = pos[1] - y
                dist = sqrt(dx**2 + dy**2)
                self.set_linear_vel(dist*4)

                time.sleep(1/RATE)

            return True
        else:
            pass


def main():
    env = Environment() # create openrave environment
    env.SetViewer('qtosg') # attach viewer (optional)

    try:
        env.Load('/home/yo/repos/openrave-playground/tools/map.env.xml') # load a simple scene
        robot = env.GetRobots()[0] # get the first robot
        
        ecro = EcroWrapper(env, robot)

        env.StartSimulation(timestep=0.001)

        time.sleep(2)  # wait to viewer

        start = np.round(ecro.get_pos()[:2])
        steps = get_route("map1", [int(start[0]), int(start[1])] , [GOAL[0]-1, GOAL[1]-1])
        steps.append(GOAL)
        print(steps)

        while steps:
            goal = steps.pop(0)
            if ecro.set_cmd_pos(goal, True):
                print("REACHED ({0}, {1})".format(str(goal[0]), str(goal[1])))

        print("\nGOAL REACHED!")

        ecro.set_linear_vel(0)
        print("BYE")

        time.sleep(5)

    finally:
        env.Destroy()

if __name__ == "__main__":
    main()
