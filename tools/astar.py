#! /usr/bin/env python

"""Implementation of A* algorithm.

Supports printing with colored map, interactive enter mode and statistical
results.
"""

import os
import sys
import argparse
import time
from math import sqrt
from operator import attrgetter

#sys.path.append('../')
from utils import Output, OUTPUT_MODE, UserInputException, Colors, CharMapCell,\
                    CharMap, Node, get_route, print_results

__author__ = "Pedro Arias Perez"


LOCAL_PATH = os.path.dirname(os.path.abspath(__file__))
FILE_NAME = LOCAL_PATH + "/assets/{0}.csv"
MAP = "map1"
START_X = 2
START_Y = 2
END_X = 7
END_Y = 2


class NodeCost(Node):
    """
    Extends Node class with cost attribute
    """
    def __init__(self, x, y, myId, parentId, cost):
        self.x = x
        self.y = y
        self.myId = myId
        self.parentId = parentId
        self.cost = cost

    def dump(self):
        """
        Prints node.
        """
        print("---------- x", str(self.x), "| y", str(self.y), "| id",\
              str(self.myId), "| parentId", str(self.parentId), "| cost", str(self.cost))


def euclidean_dist(current, goal):
    """
    Euclidean distance between two points.

    current: current point ([int, int])
    goal: second point ([int, int])

    return: distance between points (float)
    """
    return sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)


class CharMapCost(CharMap):
    """
    A map that represents the C-Space.
    """

    def __init__(self, filename, start=None, end=None):
        self.charMap = []
        self.nodes = []  # to visit
        self.closed_nodes = []  # visited
        self.n_checked = 0
        self.aux = None

        self.read(filename)
        self.start = start
        self.end = end

    @property
    def start(self):
        return self.__start

    @start.setter
    def start(self, s):
        """
        Start setter. Also adds start position as root in nodes tree.
        Raise exception if s position is non-existent or occupied.

        s: start position ([int, int])
        """
        if s is not None:
            if self.charMap[s[0]][s[1]].c == "1":  # wall
                print("[Error] Invalid start position.", file=sys.stderr)
                raise UserInputException
            try:
                self.charMap[s[0]][s[1]] = CharMapCell(3)
            except IndexError:
                print("[Error] Invalid start position.", file=sys.stderr)
                raise UserInputException
            self.nodes.append(NodeCost(s[0], s[1], 0, -2, 0))
        self.__start = s

    def check(self, cell, node):
        """
        Check if cell is end or not visited and add it to tree nodes.

        cell: current cell ([int, int])
        node: parent node (NodeCost)

        return: parent_id if goal_found else -1
        """

        self.n_checked += 1
        if( self.charMap[cell[0]][cell[1]] == '4' ):  # end
            return node.myId
        elif ( self.charMap[cell[0]][cell[1]] == '0' ):  # empty
            newNode = NodeCost(cell[0], cell[1], len(self.nodes)+len(self.closed_nodes), node.myId,
                        euclidean_dist(cell, self.start)+euclidean_dist(cell, self.end))
            self.charMap[cell[0]][cell[1]] = CharMapCell(2)
            self.nodes.append(newNode)
        return -1

    def reset(self):
        """
        Set all cells as not visited, clear tree nodes and reser checked cells counter.
        """
        self.nodes = []
        self.closed_nodes = []
        self.start = self.start
        self.end = self.end

        for row in self.charMap:
            for c in row:
                if c == "2":
                    c.c = "0"
        self.n_checked = 0

def read_from_user(m, s, e):
    """
    Gets map, start point and end point from the user.
    Raise exception if data entered is incorrect.
    Exiting loop: Crtl+D.

    m: default map (str)
    s: default start point ([int, int])
    e: default end point ([int, int])

    return: [map (str), start ([int, int]), end ([int, int])] chosen by user.
    """
    while True:
        try:
            map = input("Map ({}): ".format(m))
            if map == "":
                map = m
            charMap = CharMap(FILE_NAME.format(map))
            charMap.dump()
            break
        except EOFError:
            print("\nBye!")
            return None, None, None
        except UserInputException:
            pass

    while True:
        try:
            start_x = input("Start X ({}): ".format(s[0]))
            if start_x == "":
                start_x = s[0]
            start_y = input("Start Y ({}): ".format(s[1]))
            if start_y == "":
                start_y = s[1]
            start = [int(start_x), int(start_y)]

            charMap = CharMap(FILE_NAME.format(map), start)
            charMap.dump()
            break
        except EOFError:
            print("\nBye!")
            return None, None, None
        except UserInputException:
            pass

    while True:
        try:
            end_x = input("End X ({}): ".format(e[0]))
            if end_x == "":
                end_x = e[0]
            end_y = input("End Y ({}): ".format(e[1]))
            if end_y == "":
                end_y = e[1]
            end = [int(end_x), int(end_y)]

            charMap = CharMap(FILE_NAME.format(map), start, end)
            charMap.dump()
            break
        except EOFError:
            print("\nBye!")
            return None, None, None
        except UserInputException:
            pass

    return map, start, end


def astar(map):
    """
    Executes A* Algorithm.

    map: Map where to find the path (CharMapCost).

    return: goalParentId, id of node which found goal (id).
    """

    goalParentId = -1

    while len(map.nodes):
        print("--------------------- number of open nodes: ", len(map.nodes))
        print("--------------------- number of closed nodes: ", len(map.closed_nodes))

        node = min(map.nodes, key=attrgetter('cost'))

        map.closed_nodes.append(node)
        map.nodes.remove(node)

        map.set_current([node.x, node.y])
        map.clear_news()
        node.dump()

        # up
        tmpX = node.x - 1
        tmpY = node.y
        if map.check([tmpX, tmpY], node) != -1:
            map.dump()
            goalParentId = node.myId
            break

        # down
        tmpX = node.x + 1
        tmpY = node.y
        if map.check([tmpX, tmpY], node) != -1:
            map.dump()
            goalParentId = node.myId
            break

        # right
        tmpX = node.x
        tmpY = node.y + 1
        if map.check([tmpX, tmpY], node) != -1:
            map.dump()
            goalParentId = node.myId
            break

        # left
        tmpX = node.x
        tmpY = node.y - 1
        if map.check([tmpX, tmpY], node) != -1:
            map.dump()
            goalParentId = node.myId
            break

        map.dump()
    return goalParentId


def main(filename, start, end):
    """
    Entering method. Creates the map, execs the algorithm and prints the result.
    Raise exception if map is invalid.

    filename: map file name (str)
    start: start point ([int, int])
    end: end point ([int, int])
    """

    try:
        map = CharMapCost(filename, start, end)
    except UserInputException:
        print("[Error] Exiting..", file=stderr)
        return -1

    map.dump()

    t0 = time.time()
    goalParentId = astar(map)
    route = get_route(map.closed_nodes, goalParentId)
    tf = time.time()

    print_results([len(route), map.n_checked, round((tf-t0), 5)])


if __name__ == "__main__":
    # Command line argument parser, try: python3 a-star.py -h
    parser = argparse.ArgumentParser(description="A* Algorithm.")
    parser.add_argument('-m', '--map', metavar='MAP', dest='map', default=MAP, help='change map folder')
    parser.add_argument('-s', '--start', type=int, nargs=2, metavar='N', dest='start', default=[START_X, START_Y], help='change start point')
    parser.add_argument('-e', '--end', type=int, nargs=2, metavar='N', dest='end', default=[END_X, END_Y], help='change end point')
    parser.add_argument('-i', action='store_true', help='interactive mode (choose map, start, end...)')
    parser.add_argument('-o', type=Output, choices=Output, metavar='OUTPUT', dest='output', default=OUTPUT_MODE, help='output mode (choose from none, base, colored)')
    args = parser.parse_args()

    map = args.map
    OUTPUT_MODE = args.output
    if OUTPUT_MODE == Output.NONE:
        sys.stdout = open(os.devnull, 'w')  # silence
    start = args.start
    end = args.end
    if args.i:
        map, start, end = read_from_user(map, start, end)

    if map is not None and start is not None and end is not None:
        main(FILE_NAME.format(map), start, end)
