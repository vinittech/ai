"""
The map class contains overall mapping structure.
This makes individual map of  obstacles, dispensers, goals and blocks for each agent and create a complete representation.
At each time stamp, it also updates them at each time stamp and merge individual map of these entitites to get overall merged map.


__author__ = '{Dayyan Smith, Mayank Yadav, Vinit Jain}'
__project__='{AAIP Project SS19}'
"""

from enum import Enum

import pdb

import numpy as np
from scipy import signal
import matplotlib

matplotlib.use('TkAgg')
from matplotlib import colors
import matplotlib.animation as animation
import matplotlib.pyplot as plt
from utils import empty_perception_grid, get_entity_position, dispenser_coords_to_dispenser_grid, \
    obstacle_coords_to_obstacle_grid, goal_coords_to_goal_grid, block_coords_to_block_grid, get_top_left_goal_cell, \
    calculate_gaps
from path_planner import Astar

AGENT_VISION = 5
AGENT_VISION_DIAMETER = AGENT_VISION * 2 + 1


class Direction(Enum):
    UP = 1
    RIGHT = 2
    DOWN = 3
    LEFT = 4


class Map:
    def __init__(self, agent=None, agent_name=None, height=AGENT_VISION_DIAMETER, width=AGENT_VISION_DIAMETER,
                 msg=None):
        '''
        msg: to create map from map message
        '''

        # Create map from map message for merging
        if msg is not None:
            self._agent_name = msg.agent_name
            self.width = msg.width
            self.height = msg.height

            # 2d numpy array with obstacles information, empty 0, obstacle 1
            self.obstacles = np.array(msg.obstacles.split(","), dtype='int64').reshape((msg.height, msg.width))

            # 2d numpy array with goal information, empty 0, goal 1
            self.goals = np.array(msg.goals.split(","), dtype='int64').reshape((msg.height, msg.width))

            # 2d numpy array with dispensers marked 0, 1, 2, 3, 4, everything else -1
            self.dispensers = np.array(msg.dispensers.split(","), dtype='int64').reshape((msg.height, msg.width))

            # 2d numpy array with blocks marked 0, 1, 2, 3, 4, everything else -1
            self.blocks = np.array(msg.blocks.split(","), dtype='int64').reshape((msg.height, msg.width))

        else:
            self._agent = agent
            self._agent_name = agent_name
            self.width = width
            self.height = height
            self.obstacles = np.full((self.width, self.height), -1, dtype='int64')

            self.goals = np.full((self.width, self.height), -1, dtype='int64')

            self.dispensers = np.full((self.width, self.height), -1, dtype='int64')

            self.blocks = np.full((self.width, self.height), -1, dtype='int64')

            # TODO: Entities

            self.blocked = np.full((self.width, self.height), -1, dtype='int64')

            # 2d numpy array with obstacles, goals, dispensers, blocks (blocks take precedence over dispensers)
            self.complete_representation = np.full((self.width, self.height), -1, dtype='int64')

            # Origin in global 2d array
            self.spawn_position_global = np.array([AGENT_VISION, AGENT_VISION])

            # Relative to origin of map (agent spawn position)
            self.agent_position_relative = np.array([0, 0])

            # "global" in the 2d array
            self.agent_position_global = np.array([AGENT_VISION, AGENT_VISION])

            # For merging

            # Goal area discovered
            self.goal_area_discovered = False

            self.left_x = None
            self.right_x = None
            self.top_y = None
            self.bottom_y = None
            self.self_is_top_left = None

            # Appropriate dispenser position for task
            self.task_dispenser_position = None

            # Appropriate block position for task
            self.task_block_position = None

    def __repr__(self):
        map_representation = np.copy(self.complete_representation)
        map_representation[self.agent_position_global[1], self.agent_position_global[0]] = 7
        return map_representation.__str__()

    def _relative_to_global(self, relative_coord):
        return self.spawn_position_global + relative_coord

    def _global_to_relative(self, global_coord, agent=False):
        '''
        global_coord: (x, y)
        returns coordinates relative to spawn position (or to global agent position if agent == True)
        '''
        if agent:
            return global_coord - self.agent_position_global
        return global_coord - self.spawn_position_global

    def save(self):
        save_directory = "data/maps"
        save_path = '{}/{}.map'.format(save_directory, self._agent_name)
        np.savetxt(save_path, self.obstacles)

    def show(self):
        fig, ax = plt.subplots(1, 1)
        obstacles = np.copy(self.obstacles)
        obstacles[self.agent_position_global[1], self.agent_position_global[0]] = 3
        ax.imshow(obstacles)
        plt.show()

    def update_dimensions(self):
        self.width = self.goals.shape[1]
        self.height = self.goals.shape[0]

    def expand_map(self, direction):
        if direction == Direction.UP:
            # Goals
            new_goals = np.full((self.height + 1, self.width), -1, dtype='int64')
            new_goals[1:, :] = self.goals
            self.goals = new_goals
            # Obstacles
            new_obstacles = np.full((self.height + 1, self.width), -1, dtype='int64')
            new_obstacles[1:, :] = self.obstacles
            self.obstacles = new_obstacles
            # Dispensers
            new_dispensers = np.full((self.height + 1, self.width), -1, dtype='int64')
            new_dispensers[1:, :] = self.dispensers
            self.dispensers = new_dispensers
            # Blocks
            new_blocks = np.full((self.height + 1, self.width), -1, dtype='int64')
            new_blocks[1:, :] = self.blocks
            self.blocks = new_blocks
            self.spawn_position_global += np.array([0, 1])
        elif direction == Direction.RIGHT:
            # Goals
            new_goals = np.full((self.height, self.width + 1), -1, dtype='int64')
            new_goals[:, :-1] = self.goals
            self.goals = new_goals
            # Obstacles
            new_obstacles = np.full((self.height, self.width + 1), -1, dtype='int64')
            new_obstacles[:, :-1] = self.obstacles
            self.obstacles = new_obstacles
            # Dispensers
            new_dispensers = np.full((self.height, self.width + 1), -1, dtype='int64')
            new_dispensers[:, :-1] = self.dispensers
            self.dispensers = new_dispensers
            # Blocks
            new_blocks = np.full((self.height, self.width + 1), -1, dtype='int64')
            new_blocks[:, :-1] = self.blocks
            self.blocks = new_blocks
        elif direction == Direction.DOWN:
            # Goals
            new_goals = np.full((self.height + 1, self.width), -1, dtype='int64')
            new_goals[:-1, :] = self.goals
            self.goals = new_goals
            # Obstacles
            new_obstacles = np.full((self.height + 1, self.width), -1, dtype='int64')
            new_obstacles[:-1, :] = self.obstacles
            self.obstacles = new_obstacles
            # Dispensers
            new_dispensers = np.full((self.height + 1, self.width), -1, dtype='int64')
            new_dispensers[:-1, :] = self.dispensers
            self.dispensers = new_dispensers
            # Blocks
            new_blocks = np.full((self.height + 1, self.width), -1, dtype='int64')
            new_blocks[:-1, :] = self.blocks
            self.blocks = new_blocks
        elif direction == Direction.LEFT:
            # Goals
            new_goals = np.full((self.height, self.width + 1), -1, dtype='int64')
            new_goals[:, 1:] = self.goals
            self.goals = new_goals
            # Obstacles
            new_obstacles = np.full((self.height, self.width + 1), -1, dtype='int64')
            new_obstacles[:, 1:] = self.obstacles
            self.obstacles = new_obstacles
            # Dispensers
            new_dispensers = np.full((self.height, self.width + 1), -1, dtype='int64')
            new_dispensers[:, 1:] = self.dispensers
            self.dispensers = new_dispensers
            # Dispensers
            new_blocks = np.full((self.height, self.width + 1), -1, dtype='int64')
            new_blocks[:, 1:] = self.blocks
            self.blocks = new_blocks
            self.spawn_position_global += np.array([1, 0])
        self.update_dimensions()

    def move_agent(self, direction):
        if direction == 'n':
            self.agent_position_relative += np.array([0, -1])
            self.update_agent_position_global()
            if self.agent_position_global[1] - AGENT_VISION == -1:
                self.expand_map(direction=Direction.UP)
        elif direction == 'e':
            self.agent_position_relative += np.array([1, 0])
            self.update_agent_position_global()
            if self.agent_position_global[0] + AGENT_VISION == self.width:
                self.expand_map(direction=Direction.RIGHT)
        elif direction == 's':
            self.agent_position_relative += np.array([0, 1])
            self.update_agent_position_global()
            if self.agent_position_global[1] + AGENT_VISION == self.height:
                self.expand_map(direction=Direction.DOWN)
        elif direction == 'w':
            self.agent_position_relative += np.array([-1, 0])
            self.update_agent_position_global()
            if self.agent_position_global[0] - AGENT_VISION == -1:
                self.expand_map(direction=Direction.LEFT)
        self.update_agent_position_global()
        self.update_dimensions()

    def update(self, obstacles, goals, dispensers, blocks, entities):
        self._update_obstacles(obstacles, entities)
        self._update_goals(goals)
        self._update_dispensers(dispensers)
        self._update_blocks(blocks)
        self._update_blocked()
        self._update_complete_representation()
        self._update_goal_area_discovered()
        self.update_dimensions()

    def _update_obstacles(self, obstacles, entities):
        obstacles = [np.array([obstacle.pos.x, obstacle.pos.y]) for obstacle in obstacles]
        entities = [np.array([entity.pos.x, entity.pos.y]) for entity in entities]
        obstacles_grid = obstacle_coords_to_obstacle_grid(obstacles=obstacles, entities=entities)
        top_y = self.agent_position_global[1] - AGENT_VISION
        bottom_y = self.agent_position_global[1] + AGENT_VISION
        left_x = self.agent_position_global[0] - AGENT_VISION
        right_x = self.agent_position_global[0] + AGENT_VISION
        new_obstacles = np.maximum(obstacles_grid, self.obstacles[top_y:bottom_y + 1, left_x:right_x + 1])
        self.obstacles[top_y:bottom_y + 1, left_x:right_x + 1] = new_obstacles

    def _update_goals(self, goals):
        goals = [np.array([goal.pos.x, goal.pos.y]) for goal in goals]
        goals_grid = goal_coords_to_goal_grid(goals=goals)
        top_y = self.agent_position_global[1] - AGENT_VISION
        bottom_y = self.agent_position_global[1] + AGENT_VISION
        left_x = self.agent_position_global[0] - AGENT_VISION
        right_x = self.agent_position_global[0] + AGENT_VISION
        new_goals = np.maximum(goals_grid, self.goals[top_y:bottom_y + 1, left_x:right_x + 1])
        self.goals[top_y:bottom_y + 1, left_x:right_x + 1] = new_goals

    def _update_dispensers(self, dispensers):
        dispenser_coords = [np.array([dispenser.pos.x, dispenser.pos.y]) for dispenser in dispensers]
        dispenser_types = [dispenser.type for dispenser in dispensers]
        dispensers_grid = dispenser_coords_to_dispenser_grid(dispensers=dispenser_coords, types=dispenser_types)
        top_y = self.agent_position_global[1] - AGENT_VISION
        bottom_y = self.agent_position_global[1] + AGENT_VISION
        left_x = self.agent_position_global[0] - AGENT_VISION
        right_x = self.agent_position_global[0] + AGENT_VISION
        new_dispensers = np.maximum(dispensers_grid, self.dispensers[top_y:bottom_y + 1, left_x:right_x + 1])
        self.dispensers[top_y:bottom_y + 1, left_x:right_x + 1] = new_dispensers

    def _update_blocks(self, blocks):
        block_coords = [np.array([block.pos.x, block.pos.y]) for block in blocks]
        block_types = [block.type for block in blocks]
        blocks_grid = block_coords_to_block_grid(blocks=block_coords, types=block_types)
        top_y = self.agent_position_global[1] - AGENT_VISION
        bottom_y = self.agent_position_global[1] + AGENT_VISION
        left_x = self.agent_position_global[0] - AGENT_VISION
        right_x = self.agent_position_global[0] + AGENT_VISION

        # Remove blocks in perceivable area
        mask_indices = np.where(empty_perception_grid() == 0)
        self.blocks[top_y:bottom_y + 1, left_x:right_x + 1][mask_indices] = -1

        # Add blocks from current perception
        new_blocks = np.maximum(blocks_grid, self.blocks[top_y:bottom_y + 1, left_x:right_x + 1])
        self.blocks[top_y:bottom_y + 1, left_x:right_x + 1] = new_blocks

    def _update_blocked(self):
        '''
        mark blocks as 1, obstacles as 2
        '''
        obstacles_tmp = np.where(self.obstacles == 1, 2, self.blocks)
        self.blocked = np.maximum(self.blocks, obstacles_tmp)

    def _update_complete_representation(self):
        '''
        mark obstacles as 1, blocks as 2, dispensers as 3, goals as 4
        '''
        blocks_tmp = np.copy(self.blocks)
        blocks_tmp[blocks_tmp >= 0] += 40
        dispensers_tmp = np.copy(self.dispensers)
        dispensers_tmp[self.dispensers >= 0] += 30
        goals_tmp = np.where(self.goals == 1, 2, self.goals)
        self.complete_representation = np.amax(np.stack([self.obstacles, blocks_tmp, dispensers_tmp, goals_tmp]),
                                               axis=0)

    def _update_goal_area_discovered(self):
        self.goal_area_discovered = (self.goals == 1).sum() == 12

    def update_agent_position_global(self):
        self.agent_position_global = self._relative_to_global(self.agent_position_relative)

    def merge_with(self, other_map):
        '''
        Merges other map into self
        other_map: map object
        returns: void
        '''
        self._merge_goals(other_goals=other_map.goals)
        self._merge_obstacles(other_obstacles=other_map.obstacles)
        self._merge_blocks(other_blocks=other_map.blocks)
        self._merge_dispensers(other_dispensers=other_map.dispensers)
        self.update_agent_position_global()
        self.update_dimensions()

    def _merge_goals(self, other_goals):
        '''
        Merge two overlapping goal representations. Takes care of the more complex merging activities
        and is the basis of the merging of dispensers, blocks, obstacles
        '''
        new_shape = self._new_shape(other_goals=other_goals)
        new_goals = np.full(shape=new_shape, fill_value=-1, dtype='int64')
        self_left_gap, _, self_top_gap, _ = calculate_gaps(self.goals)
        other_left_gap, _, other_top_gap, _ = calculate_gaps(other_goals)

        # print("self.goals", self._agent_name, self.goals)
        # print("other_goals", self._agent_name, other_goals)

        new_spawn_position_global = np.array([0, 0])
        if self_left_gap >= other_left_gap:
            self.self_left_x = 0
            self.other_left_x = self_left_gap - other_left_gap
            new_spawn_position_global[0] = self.spawn_position_global[0]
        else:
            self.self_left_x = other_left_gap - self_left_gap
            self.other_left_x = 0
            new_spawn_position_global[0] = self.spawn_position_global[0] + self.self_left_x

        if self_top_gap >= other_top_gap:
            self.self_top_y = 0
            self.other_top_y = self_top_gap - other_top_gap
            new_spawn_position_global[1] = self.spawn_position_global[1]
        else:
            self.self_top_y = other_top_gap - self_top_gap
            self.other_top_y = 0
            new_spawn_position_global[1] = self.spawn_position_global[1] + self.self_top_y
        self.spawn_position_global = new_spawn_position_global

        # # Add self goals
        width_offset = self.goals.shape[1]
        height_offset = self.goals.shape[0]
        new_goals[self.self_top_y:self.self_top_y + height_offset,
        self.self_left_x:self.self_left_x + width_offset] = np.maximum(self.goals, new_goals[
                                                                                   self.self_top_y:self.self_top_y + height_offset,
                                                                                   self.self_left_x:self.self_left_x + width_offset])

        # print(self._agent_name, 'added self goals', new_goals)

        # # Add other goals
        width_offset = other_goals.shape[1]
        height_offset = other_goals.shape[0]
        new_goals[self.other_top_y:self.other_top_y + height_offset,
        self.other_left_x:self.other_left_x + width_offset] = np.maximum(other_goals, new_goals[
                                                                                      self.other_top_y:self.other_top_y + height_offset,
                                                                                      self.other_left_x:self.other_left_x + width_offset])

        # print(self._agent_name, 'added other goals', new_goals)

        self.goals = new_goals
        # print(self._agent_name, 'these are the self.goals', self.goals)

    def _merge_obstacles(self, other_obstacles):
        '''
        Merge two overlapping obstacle representations
        '''
        new_obstacles = np.full_like(self.goals, fill_value=-1, dtype='int64')
        width_offset = self.obstacles.shape[1]
        height_offset = self.obstacles.shape[0]
        new_obstacles[self.self_top_y:self.self_top_y + height_offset,
        self.self_left_x:self.self_left_x + width_offset] = self.obstacles

        width_offset = other_obstacles.shape[1]
        height_offset = other_obstacles.shape[0]
        new_obstacles[self.other_top_y:self.other_top_y + height_offset,
        self.other_left_x:self.other_left_x + width_offset] = other_obstacles

        self.obstacles = new_obstacles

    def _merge_blocks(self, other_blocks):
        '''
        Merge two overlapping block representations
        '''
        new_blocks = np.full_like(self.goals, fill_value=-1, dtype='int64')

        width_offset = self.blocks.shape[1]
        height_offset = self.blocks.shape[0]
        new_blocks[self.self_top_y:self.self_top_y + height_offset,
        self.self_left_x:self.self_left_x + width_offset] = self.blocks

        width_offset = other_blocks.shape[1]
        height_offset = other_blocks.shape[0]
        new_blocks[self.other_top_y:self.other_top_y + height_offset,
        self.other_left_x:self.other_left_x + width_offset] = other_blocks

        self.blocks = new_blocks

    def _merge_dispensers(self, other_dispensers):
        '''
        Merge two overlapping dispenser representations
        '''
        new_dispensers = np.full_like(self.goals, fill_value=-1, dtype='int64')
        width_offset = self.dispensers.shape[1]
        height_offset = self.dispensers.shape[0]
        new_dispensers[self.self_top_y:self.self_top_y + height_offset,
        self.self_left_x:self.self_left_x + width_offset] = self.dispensers
        width_offset = other_dispensers.shape[1]
        height_offset = other_dispensers.shape[0]
        new_dispensers[self.other_top_y:self.other_top_y + height_offset,
        self.other_left_x:self.other_left_x + width_offset] = other_dispensers
        self.dispensers = new_dispensers

    def _new_shape(self, other_goals):
        '''
        other_goals: 2d numpy array, arbitrary dimensions
        returns: shape of stitched map
        '''
        self_left_gap, self_right_gap, self_top_gap, self_bottom_gap = calculate_gaps(self.goals)
        other_left_gap, other_right_gap, other_top_gap, other_bottom_gap = calculate_gaps(other_goals)
        new_width = max(self_left_gap, other_left_gap) + 4 + max(self_right_gap, other_right_gap)
        new_height = max(self_top_gap, other_top_gap) + 4 + max(self_bottom_gap, other_bottom_gap)
        new_shape = (new_height, new_width)
        return new_shape

    def _get_unexplored_cell(self):
        '''
        Returns best cell to explore (global coordinates)
        '''

        # In self.goals everything that is -1 is unexplored
        pass

        # Filter
        f = np.full(shape=(3, 3), fill_value=-1, dtype='int64')
        # print(f)

        # Get discovered area (encoded in goals representation)
        discovered = np.where(self.goals == 1, 0, self.goals)

        convolved = signal.convolve2d(discovered, f, mode='same', boundary='fill', fillvalue=-1)
        # print(convolved)

        # Explore canditates
        candidates = np.where(np.logical_and(convolved > 5, convolved < 9))
        # print(candidates)
        # print(candidates[0].shape)
        # print(convolved[candidates])
        # print(convolved[candidates].shape)
        best_index = np.argmax(convolved[candidates])
        # print(best_index)
        best_coord = np.array([candidates[1][best_index], candidates[0][best_index]])
        return best_coord

    def get_explore_direction(self):
        # TODO: Replace with path planner, this naive exploration does not consider not walkable cells
        best_position = self._get_unexplored_cell()
        best_position_relative = self._global_to_relative(global_coord=best_position, agent=True)
        return self.pos_to_direction(best_position_relative)

    def update_task_dispenser_position(self):
        if self._agent.tasks.assigned_task is not None:
            print("Updating task dispenser position")
            print('the dispense map', self.complete_representation)
            # Block type needed for accomplishing task (one of 0, 1, 2, 3, ...)
            block_type = self._agent.tasks.assigned_task.requirement.block_type
            print(self._agent_name, block_type)
            candidates = get_entity_position(self.complete_representation, 30 + block_type, self.agent_position_global)
            # candidates = np.where(self.dispensers == block_type)
            print(self._agent_name, candidates)

            if len(candidates) > 1:
                best_index = 0  # TODO: get closest dispenser
                best_position = np.array([candidates[1], candidates[0]])
                print(best_position, 'is the best pos')
                self.task_dispenser_position = best_position

            else:
                print('cant find best position', candidates)
                best_position = None
                self.task_dispenser_position = best_position

    def get_dispenser_direction(self):
        astar_object = Astar()
        direction = astar_object.path_plan(self.complete_representation,
                                           (self.agent_position_global[1], self.agent_position_global[0]),
                                           (self.task_dispenser_position[1], self.task_dispenser_position[0]),
                                           30 + self._agent.tasks.assigned_task.requirement.block_type)
        if (direction is not None):
            print(direction)
            if (len(direction) > 1):
                best_coord_relative = astar_object.get_direction_from_path(
                    (self.agent_position_global[1], self.agent_position_global[0]), direction)
                print('dispenser popping')
                print(best_coord_relative)

                return best_coord_relative.pop(0)

        else:
            print('No path')

    def update_task_block_position(self):
        # Block type needed for accomplishing task (one of 0, 1, 2, 3, ...)
        block_type = self._agent.tasks.assigned_task.requirement.block_type
        candidates = np.where(self.blocks == block_type)
        # print('the block pos is', self.task_block_position)

        if len(candidates[0]) > 0:
            best_index = 0  # TODO: get closest block
            best_position = np.array([candidates[1][best_index], candidates[0][best_index]])
        else:
            best_position = None

        self.task_block_position = best_position

    def get_block_direction(self):
        best_coord_relative = self._global_to_relative(global_coord=self.task_block_position, agent=True)
        return self.pos_to_direction(best_coord_relative)

    #
    def get_goal_direction(self):
        candidates = get_entity_position(self.goals, 1, self.agent_position_global)
        # print(self.goals,self._agent_name)
        print(self._agent_name, candidates)
        best_position = None
        if len(candidates) > 1:
            best_index = 0  # TODO: get closest dispenser
            best_position = np.array([candidates[1], candidates[0]])
            print(best_position, 'is the best pos for goal', self._agent_name)

        else:
            print('the goal cells are', candidates)
            print('cant find best position for goal')
            best_position = None

        goal_position_relative = self._global_to_relative(global_coord=best_position, agent=True)
        return self.pos_to_direction(goal_position_relative)

        # astar_object=Astar()
        # if best_position is not None:
        #     direction=astar_object.path_plan(self.complete_representation,(self.agent_position_global[1],self.agent_position_global[0]),(best_position[0],best_position[1]),2)
        #     if(direction !=None):
        #         best_coord_relative =astar_object.get_direction_from_path((self.agent_position_global[1],self.agent_position_global[0]),direction)
        #         print('dispenser popping')
        #         print(best_coord_relative)
        #
        #         return best_coord_relative.pop(0)
        #     else:
        #         print('No path')
        #         return 'n'

    # def get_goal_direction(self):
    #     candidates = np.where(self.goals == 1)
    #     if len(candidates[0]) > 0:
    #         best_index = 0  # TODO: get closest goal
    #         goal_position = np.array([candidates[1][best_index], candidates[0][best_index]])
    #     else:
    #         goal_position = None
    #     goal_position_relative = self._global_to_relative(global_coord=goal_position, agent=True)
    #     return self.pos_to_direction(goal_position_relative)

    def pos_to_direction(self, pos):
        """
        Determine a direction from a given relative position
        :param pos: relative position (relative to global agent position)
        :return: direction string: 'n', 's', 'e', 'w'
        """

        if pos[0] == 0 and pos[1] > 0:
            return 's'
        elif pos[0] == 0 and pos[1] < 0:
            return 'n'
        elif pos[1] == 0 and pos[0] > 0:
            return 'e'
        elif pos[1] == 0 and pos[0] < 0:
            return 'w'
        else:  # if not directly on the same column/row as the position, we try to reduce the smaller axis value (x/y) first
            if pos[0] > pos[1]:
                if pos[1] > 0:
                    return 's'
                else:
                    return 'n'
            else:
                if pos[0] > 0:
                    return 'e'
                else:
                    return 'w'
        return None
