#!/usr/bin/env python2

import sys

sys.path.insert(0, '../..')

import rospy
import numpy as np
from mapc_ros_bridge.msg import RequestAction, GenericAction, SimStart, SimEnd, Bye

from behaviour_components.managers import Manager
from behaviour_components.activators import BooleanActivator, ThresholdActivator, EqualActivator, GreedyActivator
from behaviour_components.conditions import Negation, Condition
from behaviour_components.goals import GoalBase
from behaviour_components.condition_elements import Effect

from agent_common.behaviours import ExploreMove, RandomMove, Request, MoveToBlock, Attach, MoveToDispenser, MoveToGoal, \
    PrepareSubmission, Submit
from agent_common.providers import PerceptionProvider
from agent_common.agent_utils import get_bridge_topic_prefix
from agent_common.mapping.map import Map
from agent_common.communication.communication import Communication
import config
from agent_common.tasks.task_collection import TaskCollection
from agent_common.sensor_manager import SensorManager

import time


class MeasureDuration:
    def __init__(self):
        self.start = None
        self.end = None

    def __enter__(self):
        self.start = time.time()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.end = time.time()
        print("Total time taken: %s" % self.duration())

    def duration(self):
        return str((self.end - self.start) * 1000) + ' milliseconds'


class RhbpAgent(object):
    """
    Main class of an agent, taking care of the main interaction with the mapc_ros_bridge
    """

    def __init__(self):
        log_level = rospy.DEBUG if config.DEBUG_MODE else rospy.INFO
        rospy.logdebug("RhbpAgent::init")

        rospy.init_node('agent_node', anonymous=True, log_level=log_level)

        self._agent_name = rospy.get_param('~agent_name', 'agentA1')  # default for debugging 'agentA1'

        self._agent_topic_prefix = get_bridge_topic_prefix(agent_name=self._agent_name)

        # ensure also max_parallel_behaviours during debugging
        self._manager = Manager(prefix=self._agent_name, max_parallel_behaviours=1)

        self.behaviours = []
        self.goals = []

        self.perception_provider = PerceptionProvider()
        self.sensor_manager = SensorManager(agent=self)

        self.map = Map(agent=self, agent_name=self._agent_name)

        self.communication = Communication(agent_name=self._agent_name)
        self.map_pub = self.communication.start_map_pub_sub(callback=self._map_callback)
        self.received_map_messages = []

        self.tasks = TaskCollection()

        self._sim_started = False

        # subscribe to MAPC bridge core simulation topics
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)

        rospy.Subscriber(self._agent_topic_prefix + "end", SimEnd, self._sim_end_callback)

        rospy.Subscriber(self._agent_topic_prefix + "bye", Bye, self._bye_callback)

        rospy.Subscriber(self._agent_topic_prefix + "generic_action", GenericAction, self._callback_generic_action)

        self._received_action_response = False

    def _sim_start_callback(self, msg):
        """
        here we could also evaluate the msg in order to initialize depending on the role etc.
        :param msg:  the message
        :type msg: SimStart
        """

        if not self._sim_started:  # init only once here

            rospy.loginfo(self._agent_name + " started")

            # creating the actual RHBP model
            self._initialize_behaviour_model()

        self._sim_started = True

    def _callback_generic_action(self, msg):
        """
        ROS callback for generic actions
        :param msg: ros message
        :type msg: GenericAction
        """
        self._received_action_response = True

    def _sim_end_callback(self, msg):
        """
        :param msg:  the message
        :type msg: SimEnd
        """
        rospy.loginfo("SimEnd:" + str(msg))
        for g in self.goals:
            g.unregister()
        for b in self.behaviours:
            b.unregister()
        self._sim_started = False

    def _bye_callback(self, msg):
        """
        :param msg:  the message
        :type msg: Bye
        """
        rospy.loginfo("Simulation finished")
        rospy.signal_shutdown('Shutting down {}  - Simulation server closed'.format(self._agent_name))

    def _map_callback(self, msg):
        if self.map.goal_area_discovered and msg.agent_name != self._agent_name:
            self.received_map_messages.append(msg)

    def _action_request_callback(self, msg):
        """
        here we just trigger the decision-making and planning
        while tracking the available time and behaviour responses
        :param msg: the message
        :type msg: RequestAction
        """

        # calculate deadline for the current simulation step
        start_time = rospy.get_rostime()
        safety_offset = rospy.Duration.from_sec(0.2)  # Safety offset in seconds
        deadline_msg = rospy.Time.from_sec(msg.deadline / 1000.0)
        current_msg = rospy.Time.from_sec(msg.time / 1000.0)
        deadline = start_time + (deadline_msg - current_msg) - safety_offset

        self.perception_provider.update_perception(request_action_msg=msg)

        self._received_action_response = False

        # Our code here

        # Move agent in its current map and expand map accordingly if agent moves in simulation
        if msg.agent.last_action_result == "success" and msg.agent.last_action == "move":
            self.map.move_agent(direction=msg.agent.last_action_params[0])

        # TODO: Update attached/connected block sensors or maybe representation in map?
        if msg.agent.last_action_result == "success" and msg.agent.last_action == "attach":
            pass

        # TODO: Account for rotation?

        # Update agent's map with current perception
        self.map.update(obstacles=self.perception_provider.obstacles, goals=self.perception_provider.goals,
                        dispensers=self.perception_provider.dispensers, blocks=self.perception_provider.blocks,
                        entities=self.perception_provider.entities)

        # Measure merge duration
        with MeasureDuration() as m:
            if self.map.goal_area_discovered:
                # print("{}:: Goal area discovered".format(self._agent_name))
                # Send own map
                self.communication.send_map(publisher=self.map_pub, obstacles=self.map.obstacles, goals=self.map.goals,
                                            blocks=self.map.blocks, dispensers=self.map.dispensers,
                                            width=self.map.width, height=self.map.height)
                # print("{}:: Sent map".format(self._agent_name))

                # Process received maps
                for map_message in self.received_map_messages:
                    other_map = Map(msg=map_message)
                    self.map.merge_with(other_map=other_map)
                    self.map._update_complete_representation()

                    self.received_map_messages.remove(map_message)

        # print("{}:: Number of received map messages: {}".format(self._agent_name, len(self.received_map_messages)))
        # print("{}\n{}\n{}".format(self._agent_name, self.map.goals.shape, self.map))
        # print(self._agent_name, 'agent_position_global', self.map.agent_position_global)

        # Tasks

        # TODO: Update attached/connected block sensors or maybe representation in map?
        if msg.agent.last_action_result == "success" and msg.agent.last_action == "submit":
            self.tasks.assigned_task.completed = True
            self.map.task_block_position = None
            self.map.task_dispenser_position = None
            self.sensor_manager.reset_task_sensors()

        self.tasks.update_tasks(tasks_percept=self.perception_provider.tasks,
                                simulation_step=self.perception_provider.simulation_step)
        if self.tasks.assigned_task is None or self.tasks.assigned_task.completed:
            self.tasks.update_assigned_task()

        self.map.update_task_dispenser_position()
        self.map.update_task_block_position()

        # print("Assigned task: {}".format(self.tasks.assigned_task))
        # print("x:{}, y:{}, type:{}".format(self.tasks.assigned_task.requirement.x, self.tasks.assigned_task.requirement.y, self.tasks.assigned_task.requirement.block_type))
        # print('task_dispenser_position: {}'.format(self.map.task_dispenser_position))

        # Update sensors
        self.sensor_manager.update_sensors(request_action_message=msg)
        # print('dispenser in map?', self.sensor_manager.dispenser_in_map_sensor._value)
        # print('dispenser distance?', self.sensor_manager.dispenser_distance_sensor._value)
        # print('block in map?', self.sensor_manager.block_in_map_sensor._value)
        # print('block distance?', self.sensor_manager.block_distance_sensor._value)
        # print('block attached?', self.sensor_manager.block_attached_sensor._value)
        # print('next to dispenser?', self.sensor_manager.next_to_dispenser_sensor._value)
        # print('next to block?', self.sensor_manager.next_to_block_sensor._value)
        # print('correct pattern position?', self.sensor_manager.correct_pattern_position_sensor._value)
        # print('on goal?', self.sensor_manager.on_goal_sensor._value)

        # /Our code here

        # self._received_action_response is set to True if a generic action response was received(send by any behaviour)
        while not self._received_action_response and rospy.get_rostime() < deadline:
            # wait until this agent is completely initialised
            if self._sim_started:  # we at least wait our max time to get our agent initialised
                # action send is finally triggered by a selected behaviour
                self._manager.step(guarantee_decision=True)
            else:
                rospy.sleep(0.1)

        if self._received_action_response:  # One behaviour replied with a decision
            duration = rospy.get_rostime() - start_time
            rospy.logdebug("%s: Decision-making duration %f", self._agent_name, duration.to_sec())

        elif not self._sim_started:  # Agent was not initialised in time
            rospy.logwarn("%s idle_action(): sim not yet started", self._agent_name)
        else:  # Our decision-making has taken too long
            rospy.logwarn("%s: Decision-making timeout", self._agent_name)

    def _initialize_behaviour_model(self):
        """
        This function initialises the RHBP behaviour/goal model.
        """

        # RANDOM MOVE
        random_move = RandomMove(name="random_move", agent_name=self._agent_name)
        self.behaviours.append(random_move)
        random_move.add_effect(Effect(self.sensor_manager.dispenser_distance_sensor.name, indicator=True))
        random_move.add_precondition(
            Condition(self.sensor_manager.dispenser_in_map_sensor, BooleanActivator(desiredValue=False)))

        # Exploration
        # explore_move = ExploreMove(name="explore_move", agent_name=self._agent_name, agent_map=self.map)
        # self.behaviours.append(explore_move)
        # explore_move.add_effect(Effect(self.perception_provider.dispenser_visible_sensor.name, indicator=True))

        # MOVE TO DISPENSER
        move_to_dispenser = MoveToDispenser(name="move_to_dispenser", perception_provider=self.perception_provider,
                                            agent_name=self._agent_name, agent_map=self.map)
        self.behaviours.append(move_to_dispenser)
        move_to_dispenser.add_effect(
            Effect(self.sensor_manager.next_to_dispenser_sensor.name, indicator=True))
        # move_to_dispenser.add_effect(
        #     Effect(self.sensor_manager.pseudo_sensor.name, indicator=1, sensor_type=float))
        move_to_dispenser.add_precondition(
            Condition(self.sensor_manager.dispenser_in_map_sensor, BooleanActivator(desiredValue=True)))
        move_to_dispenser.add_precondition(Condition(self.sensor_manager.next_to_dispenser_sensor,
                                                     BooleanActivator(desiredValue=False)))

        # # Our simple goal is to move to the dispenser
        # move_to_dispenser_goal = GoalBase("move_to_dispenser_goal", permanent=True,
        #                          conditions=[Condition(self.sensor_manager.pseudo_sensor, GreedyActivator())],
        #                          planner_prefix=self._agent_name)
        # self.goals.append(move_to_dispenser_goal)

        # Request a block if close enough
        request = Request(name="request", perception_provider=self.perception_provider, agent_name=self._agent_name,
                          agent_map=self.map)
        self.behaviours.append(request)
        # request.add_effect(
        #     Effect(self.sensor_manager.pseudo_sensor.name, indicator=+1, sensor_type=float))
        request.add_effect(Effect(self.sensor_manager.next_to_block_sensor.name, indicator=True))
        request.add_precondition(Condition(self.sensor_manager.next_to_dispenser_sensor,
                                           BooleanActivator(desiredValue=True)))

        # # Our simple goal is to request a block
        # request_goal = GoalBase("request_goal", permanent=True,
        #                          conditions=[Condition(self.sensor_manager.pseudo_sensor, GreedyActivator())],
        #                          planner_prefix=self._agent_name)
        # self.goals.append(request_goal)

        # # Move to nearby block
        # move_to_block = MoveToBlock(name="move_to_block", perception_provider=self.perception_provider,
        #                                     agent_name=self._agent_name, agent_map=self.map)
        # self.behaviours.append(move_to_block)
        # move_to_block.add_effect(
        #     Effect(self.sensor_manager.block_distance_sensor.name, indicator=-1, sensor_type=float))
        # # move_to_block.add_effect(
        # #     Effect(self.sensor_manager.pseudo_sensor.name, indicator=1, sensor_type=float))
        # move_to_block.add_precondition(
        #     Condition(self.sensor_manager.block_in_map_sensor, BooleanActivator(desiredValue=True)))
        # move_to_block.add_precondition(Condition(self.sensor_manager.block_distance_sensor,
        #                                     ThresholdActivator(isMinimum=True, thresholdValue=2)))

        # # Our simple goal is to move to a block
        # move_to_block_goal = GoalBase("move_to_block_goal", permanent=True,
        #                          conditions=[Condition(self.sensor_manager.pseudo_sensor, GreedyActivator())],
        #                          planner_prefix=self._agent_name)
        # self.goals.append(move_to_block_goal)

        # ATTACH
        attach = Attach(name="attach", perception_provider=self.perception_provider, agent_name=self._agent_name,
                        agent_map=self.map)
        self.behaviours.append(attach)
        # attach.add_effect(Effect(self.sensor_manager.pseudo_sensor.name, indicator=+1, sensor_type=float))
        attach.add_effect(Effect(self.sensor_manager.block_attached_sensor.name, indicator=True))
        attach.add_precondition(
            Condition(self.sensor_manager.next_to_block_sensor, BooleanActivator(desiredValue=True)))

        # # Our simple goal is to attach to a block
        # attach_goal = GoalBase("attach_goal", permanent=True,
        #                          conditions=[Condition(self.sensor_manager.pseudo_sensor, GreedyActivator())],
        #                          planner_prefix=self._agent_name)
        # self.goals.append(attach_goal)

        # MOVE TO GOAL
        move_to_goal = MoveToGoal(name="move_to_goal", perception_provider=self.perception_provider,
                                  agent_name=self._agent_name, agent_map=self.map)
        self.behaviours.append(move_to_goal)
        # move_to_goal.add_effect(Effect(self.sensor_manager.pseudo_sensor.name, indicator=+1, sensor_type=float))
        move_to_goal.add_effect(Effect(self.sensor_manager.on_goal_sensor.name, indicator=True))
        move_to_goal.add_precondition(
            Condition(self.sensor_manager.block_attached_sensor, BooleanActivator(desiredValue=True)))

        # # Our simple goal is to attach to a block
        # move_to_goal_goal = GoalBase("move_to_goal_goal", permanent=True,
        #                          conditions=[Condition(self.sensor_manager.pseudo_sensor, GreedyActivator())],
        #                          planner_prefix=self._agent_name)
        # self.goals.append(move_to_goal_goal)

        # PREPARE SUBMISSION
        prepare_submission = PrepareSubmission(name="prepare_submission", perception_provider=self.perception_provider,
                                               agent_name=self._agent_name)
        self.behaviours.append(prepare_submission)
        prepare_submission.add_effect(Effect(self.sensor_manager.correct_pattern_position_sensor.name, indicator=True))
        # prepare_submission.add_effect(Effect(self.sensor_manager.pseudo_sensor.name, indicator=+1, sensor_type=float))
        prepare_submission.add_precondition(
            Condition(self.sensor_manager.on_goal_sensor, BooleanActivator(desiredValue=True)))
        prepare_submission.add_precondition(
            Condition(self.sensor_manager.block_attached_sensor, BooleanActivator(desiredValue=True)))

        # # Our simple goal is to rotate the pattern into the correct position
        # prepare_submission_goal = GoalBase("prepare_submission_goal", permanent=True,
        #                          conditions=[Condition(self.sensor_manager.pseudo_sensor, GreedyActivator())],
        #                          planner_prefix=self._agent_name)
        # self.goals.append(prepare_submission_goal)

        # SUBMIT
        submit = Submit(name="submit", perception_provider=self.perception_provider,
                        agent_name=self._agent_name, agent=self)
        self.behaviours.append(submit)
        submit.add_effect(Effect(self.sensor_manager.pseudo_sensor.name, indicator=+1, sensor_type=float))
        submit.add_precondition(Condition(self.sensor_manager.on_goal_sensor, BooleanActivator(desiredValue=True)))
        submit.add_precondition(
            Condition(self.sensor_manager.correct_pattern_position_sensor, BooleanActivator(desiredValue=True)))

        # Our simple goal is to submit a task
        submit_goal = GoalBase("submit_goal", permanent=True,
                               conditions=[Condition(self.sensor_manager.pseudo_sensor, GreedyActivator())],
                               planner_prefix=self._agent_name)
        self.goals.append(submit_goal)

        # # Our simple goal is to create more and more blocks
        # dispense_goal = GoalBase("dispensing", permanent=True,
        #                          conditions=[Condition(self.perception_provider.number_of_blocks_sensor, GreedyActivator())],
        #                          planner_prefix=self._agent_name)
        # self.goals.append(dispense_goal)


if __name__ == '__main__':
    try:
        rhbp_agent = RhbpAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
