"""
Utility function for map.py to use for general purpose.

__author__ = '{Dayyan Smith, Mayank Yadav, Vinit Jain}'
__project__='{AAIP Project SS19}'
"""

import scipy.spatial
import numpy as np

AGENT_VISION = 5
AGENT_VISION_DIAMETER = AGENT_VISION * 2 + 1


def empty_perception_grid():
    # Mark cells in visible range as empty
    agent_position = np.array([AGENT_VISION, AGENT_VISION])
    perception_grid = np.full((AGENT_VISION_DIAMETER, AGENT_VISION_DIAMETER), -1, dtype='int64')
    for i in range(perception_grid.shape[0]):
        for j in range(perception_grid.shape[1]):
            current_element = np.array([i, j])
            if scipy.spatial.distance.cityblock(agent_position, current_element) <= AGENT_VISION:
                perception_grid[i, j] = 0
    return perception_grid


# Coords of agent perception to 2d array
def obstacle_coords_to_obstacle_grid(obstacles=None, entities=None):
    '''
    Convert coordinates of obstacles into grid

    obstacle_coords: obstacle coordinates, marked as 1
    '''
    obstacle_grid = empty_perception_grid()
    agent_position = np.array([AGENT_VISION, AGENT_VISION], dtype='int64')  # global position in obstacle_grid
    for o in obstacles:
        obstacle_position = np.array([o[0], o[1]]) + agent_position
        obstacle_grid[obstacle_position[1], obstacle_position[0]] = 1

    # for e in entities:
    #     if e[0] == 0 and e[1] == 0:
    #         # print(es[0], e[1], "is the agent value")
    #         continue
    #     else:
    #         entity_position = np.array([e[0], e[1]]) + agent_position
    #
    #         obstacle_grid[entity_position[1], entity_position[0]] = 1

    return obstacle_grid


def goal_coords_to_goal_grid(goals=None):
    '''
    Convert coordinates of goals into grid

    goal_coords: goal coordinates, marked as 2
    '''
    goal_grid = empty_perception_grid()
    agent_position = np.array([AGENT_VISION, AGENT_VISION], dtype='int64')  # global position in goal_grid
    for g in goals:
        goal_position = np.array([g[0], g[1]]) + agent_position
        goal_grid[goal_position[1], goal_position[0]] = 1
    return goal_grid


# Coords of agent perception to 2d array
def dispenser_coords_to_dispenser_grid(dispensers=None, types=None):
    '''
    Convert coordinates of dispensers into grid

    dispensers: dispenser coordinates, marked as 1
    '''
    dispenser_grid = np.full((AGENT_VISION_DIAMETER, AGENT_VISION_DIAMETER), -1, dtype='int64')
    agent_position = np.array([AGENT_VISION, AGENT_VISION], dtype='int64')  # global position in dispenser_grid
    for d, t in zip(dispensers, types):
        dispenser_position = np.array([d[0], d[1]]) + agent_position
        t = int(t[1:])  # remove 'b' (from b0, b1, b2, for example
        dispenser_grid[dispenser_position[1], dispenser_position[0]] = t
    return dispenser_grid


# Coords of agent perception to 2d array
def block_coords_to_block_grid(blocks=None, types=None):
    '''
    Convert coordinates of blocks into grid

    blocks: blocks coordinates, marked as 1
    '''
    block_grid = np.full((AGENT_VISION_DIAMETER, AGENT_VISION_DIAMETER), -1, dtype='int64')
    agent_position = np.array([AGENT_VISION, AGENT_VISION], dtype='int64')  # global position in dispenser_grid
    for b, t in zip(blocks, types):
        block_position = np.array([b[0], b[1]]) + agent_position
        t = int(t[1:])  # remove 'b' (from b0, b1, b2, for example
        block_grid[block_position[1], block_position[0]] = t
    return block_grid


def get_top_left_goal_cell(goals):
    '''
    goals: goal array
    '''
    goal_indices = np.where(goals == 1)
    top_left_goal_cell = np.array([goal_indices[1][0], goal_indices[0][0]])
    return top_left_goal_cell


def calculate_gaps(goals):
    '''
    m: goal array
    returns: (left_gap, right_gap, top_gap, bottom_gap)
    '''
    top_left_goal_cell = get_top_left_goal_cell(goals)
    left_gap = top_left_goal_cell[0] - 1
    right_gap = goals.shape[1] - left_gap - 4
    top_gap = top_left_goal_cell[1]
    bottom_gap = goals.shape[0] - top_gap - 4
    return (left_gap, right_gap, top_gap, bottom_gap)


def get_agent_position(graph_map, agent_val=7):
    """
    Since we know that our agent is always marked as 5, hence this will return the current agent position
    :param map: the local map
    :return: (x,y) of the agent
    """
    agent = np.where(graph_map == agent_val)

    if (len(agent[0]) == 0):
        return (0, False)
    else:
        return ((int(agent[0]), int(agent[1])), True)


def get_entity_position(graph_map, entity_value, agent_position):
    """
    entity value is the value of particular thing in the map.

    :param graph_map: local or dynamic map
    :param entity_value: any of above mentione value
    :return: nearest entity position to the graph else return None
    """
    position = np.where(graph_map == entity_value)

    if (len(position[0]) == 0):  # there is no entity in the map
        return [0]
    else:
        if (len(position[0]) == 1):  # there is only one entity in the map

            return np.array([int(position[0]), int(position[1])])  # return the positon as well as the value
        else:
            if (True):

                nearest_position = np.array([0, 0])
                distance = 40000
                for i in range(len(position[0])):
                    dist = abs(position[0][i] - agent_position[1]) + abs(position[1][i] - agent_position[0])
                    if (dist < distance):
                        nearest_position[0] = position[0][i]
                        nearest_position[1] = position[1][i]
                        distance = dist

            return nearest_position
