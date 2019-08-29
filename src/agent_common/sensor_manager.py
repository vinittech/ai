"""
Sensor manager module contains the sensor information and update.
__author__ = '{Dayyan Smith, Mayank Yadav, Vinit Jain}'
__project__='{AAIP Project SS19}'
"""

from __future__ import division  # force floating point division when using plain /
import rospy
import sys
import numpy as np
import math

from behaviour_components.sensors import Sensor

from agent_common.agent_utils import relative_euclidean_distance


class SensorManager():
    def __init__(self, agent):
        self._agent = agent
        self.dispenser_in_map_sensor = Sensor(name="dispenser_in_map", initial_value=False)
        self.dispenser_distance_sensor = Sensor(name="dispenser_distance", initial_value=sys.maxint)
        self.block_in_map_sensor = Sensor(name="block_in_map_sensor", initial_value=False)
        self.block_distance_sensor = Sensor(name="block_distance", initial_value=sys.maxint)
        self.next_to_block_sensor = Sensor(name="next_to_block", initial_value=False)
        self.next_to_dispenser_sensor = Sensor(name="next_to_block", initial_value=False)
        self.block_attached_sensor = Sensor(name="block_attached", initial_value=False)
        self.correct_pattern_position_sensor = Sensor(name="correct_pattern_position", initial_value=False)
        self.on_goal_sensor = Sensor(name="on_goal", initial_value=False)
        self.pseudo_sensor = Sensor(name="pseudo_sensor", initial_value=0)  # for testing

    def update_sensors(self, request_action_message):
        self._update_dispenser_in_map_sensor()
        self._update_dispenser_distance_sensor()
        self._update_block_in_map_sensor()
        self._update_block_distance_sensor()
        self._update_next_to_block_sensor()
        self._update_next_to_dispenser_sensor()
        self._update_block_attached_sensor(request_action_message)
        self._update_correct_pattern_position_sensor(request_action_message)
        self._update_on_goal_sensor(request_action_message)

    def _update_dispenser_in_map_sensor(self):
        if self._agent.map.task_dispenser_position is not None:
            self.dispenser_in_map_sensor.update(newValue=True)
        else:
            self.dispenser_in_map_sensor.update(newValue=False)
        self.dispenser_in_map_sensor.sync()

    def _update_dispenser_distance_sensor(self):
        if self._agent.map.task_dispenser_position is not None:
            distance = self.euclidean_distance(self._agent.map.agent_position_global,
                                               self._agent.map.task_dispenser_position)
            self.dispenser_distance_sensor.update(newValue=distance)
            self.dispenser_distance_sensor.sync()

    def _update_block_in_map_sensor(self):
        if self._agent.map.task_block_position is not None:
            self.block_in_map_sensor.update(newValue=True)
        else:
            self.block_in_map_sensor.update(newValue=False)
        self.dispenser_in_map_sensor.sync()

    def _update_block_distance_sensor(self):
        if self._agent.map.task_block_position is not None:
            distance = self.euclidean_distance(self._agent.map.agent_position_global,
                                               self._agent.map.task_block_position)
            self.block_distance_sensor.update(newValue=distance)
            self.block_distance_sensor.sync()

    def _update_next_to_block_sensor(self):
        if self._agent.map.task_block_position is not None:
            distance = self.euclidean_distance(self._agent.map.agent_position_global,
                                               self._agent.map.task_block_position)
            if distance == 1:
                self.next_to_block_sensor.update(newValue=True)
                self.next_to_block_sensor.sync()
            else:
                self.next_to_block_sensor.update(newValue=False)
                self.next_to_block_sensor.sync()
        else:
            self.next_to_block_sensor.update(newValue=False)
            self.next_to_block_sensor.sync()

    def _update_next_to_dispenser_sensor(self):
        if self._agent.map.task_dispenser_position is not None:
            distance = self.euclidean_distance(self._agent.map.agent_position_global,
                                               self._agent.map.task_dispenser_position)
            if distance == 1:
                self.next_to_dispenser_sensor.update(newValue=True)
                self.next_to_dispenser_sensor.sync()
            else:
                self.next_to_dispenser_sensor.update(newValue=False)
                self.next_to_dispenser_sensor.sync()

    def _update_block_attached_sensor(self, request_action_message):
        if request_action_message.agent.last_action_result == "success" and request_action_message.agent.last_action == "attach":
            self.block_attached_sensor.update(newValue=True)
            self.block_attached_sensor.sync()
        elif request_action_message.agent.last_action_result == "success" and request_action_message.agent.last_action == "submit":
            self.block_attached_sensor.update(newValue=False)
            self.block_attached_sensor.sync()
            # print(request_action_message.last_action,'Last action')

    def _update_correct_pattern_position_sensor(self, request_action_message):
        if self._agent.tasks.assigned_task is not None:
            task_x = self._agent.tasks.assigned_task.requirement.x
            task_y = self._agent.tasks.assigned_task.requirement.y
            blocks = [[block.pos.x, block.pos.y] for block in request_action_message.blocks]
            if [task_x, task_y] in blocks:
                self.correct_pattern_position_sensor.update(newValue=True)
                self.correct_pattern_position_sensor.sync()
            else:
                self.correct_pattern_position_sensor.update(newValue=False)
                self.correct_pattern_position_sensor.sync()

    def _update_on_goal_sensor(self, request_action_message):
        goals = [[goal.pos.x, goal.pos.y] for goal in request_action_message.goals]
        if [0, 0] in goals:
            self.on_goal_sensor.update(newValue=True)
            self.on_goal_sensor.sync()
        else:
            self.on_goal_sensor.update(newValue=False)
            self.on_goal_sensor.sync()

    # For resetting sensors after completing a task
    def reset_task_sensors(self):
        self.dispenser_in_map_sensor.update(newValue=False)
        self.dispenser_in_map_sensor.sync()
        self.dispenser_distance_sensor.update(newValue=sys.maxint)
        self.dispenser_distance_sensor.sync()

    def euclidean_distance(self, pos1, pos2):
        """
        Calculate the euclidean distance between two positions
        :param pos1: position 1 (x, y)
        :param pos2: position 2 (x, y)
        :return: euclidean distance
        """
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)
