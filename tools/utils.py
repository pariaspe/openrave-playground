#! /usr/bin/env python

"""Utils for graph searching algorithms."""

import sys
from enum import Enum

__author__ = "Pedro Arias Perez"


class Output(Enum):
    """
    Enumerate to change the standard output format.
    NONE: sys.stdout = os.devnull -> no output is printed
    BASE: cells are printed with their number codes (0 -> empty, 1 -> wall...)
    COLORED: cells are printed with thier color codes (white -> empty, black -> wall...)
    """
    NONE = 'none'
    BASE = 'base'
    COLORED = 'colored'


OUTPUT_MODE = Output.COLORED


class UserInputException(Exception):
    """Custom exception to raise when user usage error."""
    pass


class Colors:
    """
    ANSI color codes (source: https://en.wikipedia.org/wiki/ANSI_escape_code).
    """

    ESC = "\033"
    BLINK = "5"
    FG_BLACK = "30"
    FG_RED = "31"
    FG_GREEN = "32"
    FG_YELLOW = "33"
    FG_BLUE = "34"
    FG_MAGENTA = "35"
    FG_CYAN = "36"
    FG_WHITE = "37"
    BG_BLACK = "40"
    BG_RED = "41"
    BG_GREEN = "42"
    BG_YELLOW = "43"
    BG_BLUE = "44"
    BG_MAGENTA = "45"
    BG_CYAN = "46"
    BG_WHITE = "47"


class CharMapCell:
    """
    Wrapper for chars that allows formatting at printing.
    """

    RESET = Colors.ESC + "[0m"
    EMPTY = Colors.ESC + "[" + Colors.FG_WHITE + ";" + Colors.BG_WHITE + "m"
    WALL = Colors.ESC + "[" + Colors.FG_BLACK + ";" + Colors.BG_BLACK + "m"
    NEW = Colors.ESC + "[" + Colors.FG_CYAN + ";" + Colors.BG_CYAN + "m"
    VISITED = Colors.ESC + "[" + Colors.FG_BLUE + ";" + Colors.BG_BLUE + "m"
    C_VISITED = Colors.ESC + "[" + Colors.BLINK + ";" + Colors.BG_BLUE + "m"
    START = Colors.ESC + "[" + Colors.FG_GREEN + ";" + Colors.BG_GREEN + "m"
    C_START = Colors.ESC + "[" + Colors.BLINK + ";" + Colors.BG_GREEN + "m"
    END = Colors.ESC + "[" + Colors.FG_RED + ";" + Colors.BG_RED + "m"
    C_END = Colors.ESC + "[" + Colors.BLINK + ";" + Colors.BG_RED + "m"


    def __init__(self, c):
        self.c = str(c)
        self.is_current = False
        self.is_new = True if self.c == "2" else False

    def __eq__(self, o):
        if isinstance(o, CharMap):
            return self.c == o.c
        elif isinstance(o, int):
            return self.c == str(o)
        elif isinstance(o, str):
            return self.c == o
        else:
            return False

    def __add__(self, o):
        if isinstance(o, CharMap):
            return  str(self) + str(o)
        else:
            raise TypeError("invalid operation between", type(self), "and", type(o))

    def __str__(self):
        if OUTPUT_MODE == Output.BASE:  # just char printing
            return self.c
        elif OUTPUT_MODE == Output.COLORED:  # colored printing
            if self.c == "0":
                return self.EMPTY + self.c + self.RESET
            elif self.c  == "1":
                return self.WALL + self.c + self.RESET
            elif self.c == "2":
                if self.is_new:
                    return self.NEW + self.c + self.RESET
                if self.is_current:
                    return self.C_VISITED + "X" + self.RESET
                return self.VISITED + self.c + self.RESET
            elif self.c == "3":
                if self.is_current:
                    return self.C_START + "X" + self.RESET
                return self.START + self.c + self.RESET
            elif self.c == "4":
                if self.is_current:
                    return self.C_END + "X" + self.RESET
                return self.END + self.c + self.RESET
            else:
                return self.c
        else:
            return ""


class CharMap:
    """
    A map that represents the C-Space.
    """

    def __init__(self, filename, start=None, end=None):
        self.charMap = []
        self.nodes = []
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
            self.nodes.append(Node(s[0], s[1], 0, -2))
        self.__start = s

    @property
    def end(self):
        return self.__end

    @end.setter
    def end(self, e):
        """
        End setter.
        Raise exception if e position is non-existent or occupied.

        e: end position ([int, int])
        """
        if e is not None:
            if self.charMap[e[0]][e[1]].c in ["1", "3"]:  # wall or start
                print("[Error] Invalid end position.", file=sys.stderr)
                raise UserInputException
            try:
                self.charMap[e[0]][e[1]] = CharMapCell(4)
            except IndexError:
                print("[Error] Invalid end position.", file=sys.stderr)
                raise UserInputException
        self.__end = e

    def read(self, filename):
        """
        Reads map from file and save it at charMap attribute.
        Raise exception if map file is not found.

        filename: path to file (str).
        """
        try:
            with open(filename) as f:
                line = f.readline()
                while line:
                    charLine = line.strip().split(',')
                    l = []
                    for c in charLine:
                        l.append(CharMapCell(c))
                    self.charMap.append(l)
                    line = f.readline()
        except FileNotFoundError:
            print("[Error] Map not found.", file=sys.stderr)
            raise UserInputException

    def dump(self):
        """
        Prints map.
        """

        for line in self.charMap:
            l = ""
            for char in line:
                l += str(char)
            print(l)
        print()  # empty line behind map

    def check(self, cell, node):
        """
        Check if cell is end or not visited and add it to tree nodes.

        cell: current cell ([int, int])
        node: parent node (Node)

        return: parent_id if goal_found else -1
        """

        self.n_checked += 1
        if( self.charMap[cell[0]][cell[1]] == '4' ):  # end
            return node.myId
        elif ( self.charMap[cell[0]][cell[1]] == '0' ):  # empty
            newNode = Node(cell[0], cell[1], len(self.nodes), node.myId)
            self.charMap[cell[0]][cell[1]] = CharMapCell(2)
            self.nodes.append(newNode)
        return -1

    def set_current(self, cell):
        """
        Set cell as current for displaying reasons.
        """

        if self.aux is not None:
            self.charMap[self.aux[0]][self.aux[1]].is_current = False
        self.aux = cell
        self.charMap[cell[0]][cell[1]].is_current = True

    def clear_news(self):
        """
        Set all cells as not news for displaying reasons.
        """

        for row in self.charMap:
            for c in row:
                c.is_new = False

    def reset(self):
        """
        Set all cells as not visited, clear tree nodes and reser checked cells counter.
        """
        self.nodes = []
        self.start = self.start
        self.end = self.end

        for row in self.charMap:
            for c in row:
                if c == "2":
                    c.c = "0"
        self.n_checked = 0


class Node:
    """
    Node: visited cell of the map.
    """

    def __init__(self, x, y, myId, parentId):
        self.x = x
        self.y = y
        self.myId = myId
        self.parentId = parentId

    def dump(self):
        """
        Prints node.
        """
        print("---------- x", str(self.x), "| y", str(self.y), "| id",\
              str(self.myId), "| parentId", str(self.parentId))


def get_route(nodes, goalParentId):
    """
    Goes through the nodes tree to find the path found.

    nodes: tree node ([*Node])
    goalParentId: id of node which foun goal (int)

    return: route, list of nodes from start to goal ([*Node]).
    """
    print("%%%%%%%%%%%%%%%%%%%")
    route = []
    ok = False
    while not ok:
        for node in nodes:
            if( node.myId == goalParentId ):
                route.insert(0, node)
                node.dump()
                goalParentId = node.parentId
                if( goalParentId == -2):
                    print("%%%%%%%%%%%%%%%%%")
                    ok = True
    return route

def print_results(results):
    """
    Prints the statistical results obtained with a certain format.

    results: route length, numer of cells accessed and execution time ([int, int, float])
    """
    print()
    print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
    print("%%           RESULTS            %%")
    print("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
    print()
    print("Route \t Cells \t")
    print("Length\tChecked\t  Time")
    print("--------------------------------")
    print("{0}\t{1}\t{2}".format(*results))
    print()


if __name__ == "__main__":
    print("[Error] Error usage.", file=sys.stderr)
