unknown = -1
empty = 0
block = 1
obstacle = 2
dispenser = 3
goal = 4
agent = 5

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import config

"""
This submodule contains a path plannar based on A star algorithm. 
It takes the initial and final position and returns the path in term of direction.
Adapted from - https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
and made changes to take sets as closed list and hash the nodes for faster processing.

__author__ = '{Mayank Yadav}'
__project__='{AAIP Project SS19}'
"""


# TODO - make blocks attached as a part of agent covering whole workspace so that it does not get stuck anywhere.

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0  # distance between current node and start node
        self.h = 0  # estimated distance from current node to end node
        self.f = 0  # total cost of node (g+h)

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):  # <-- added a hash method
        return hash(self.position)


class Astar(object):

    def path_plan(self, maze, start, end, entity):
        """

        :param maze: the dynamic map or local map of the agent
        :param start: the current position of the agent
        :param end: the goal positon. Can be position of dispenser, goal or nearest agent to attach
        :param entity: the entity value of end point.
        :return: the list containing the path
        """
        # maze[start[0]][start[1]]=0
        # maze[end[0]][end[1]]=0

        maze[start[0]][start[1]] = 7

        # Create start and end node
        start_node = Node(None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, end)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = set()

        # Add the start node
        open_list.append(start_node)

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            current_node = open_list[0]
            current_index = 0
            for index, item in enumerate(open_list):
                if item.f < current_node.f:
                    current_node = item
                    current_index = index

            # Pop current off open list, add to closed list
            open_list.pop(current_index)
            closed_list.add(current_node)

            # Found the goal
            if current_node == end_node:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.position)
                    current = current.parent

                return path[::-1]  # Retpathurn reversed path

            # Generate children
            children = []
            for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:  # Adjacent squares

                # Get node position
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

                # Make sure within range
                if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (
                        len(maze[len(maze) - 1]) - 1) or node_position[1] < 0:
                    continue

                # Make sure walkable terrain
                if maze[node_position[0]][node_position[1]] != 0 and maze[node_position[0]][
                    node_position[1]] != entity and maze[node_position[0]][node_position[1]] != 7 and \
                        maze[node_position[0]][node_position[1]] != 2:
                    continue

                # Create new node
                new_node = Node(current_node, node_position)

                # Append
                children.append(new_node)

            # Loop through children
            for child in children:

                if child in closed_list:
                    continue

                # Create the f, g, and h values
                child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                        (child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for open_node in open_list:
                    if child == open_node and child.g > open_node.g:
                        continue

                # Add the child to the open list
                open_list.append(child)

    def get_direction_from_path(self, start, path):

        direction_path = []
        x_initial = int(start[0])
        y_initial = int(start[1])

        for i in range(len(path)):
            x_final = path[i][0]
            y_final = path[i][1]
            if ((x_final - x_initial) == 0 and (y_final - y_initial) == 0):
                continue
            else:
                direction_path.append(self.pos_to_direction((y_final - y_initial), (x_final - x_initial)))
                x_initial = x_final
                y_initial = y_final

        return direction_path

    def pos_to_direction(self, x, y):
        """
        Determine a direction from a given relative position
        :param pos: relative position with x and y
        :return: direction string like 'n', 's', 'e', 'w'
        """

        if x == 0 and y > 0:
            return 's'
        elif x == 0 and y < 0:
            return 'n'
        elif y == 0 and x > 0:
            return 'e'
        elif y == 0 and x < 0:
            return 'w'
