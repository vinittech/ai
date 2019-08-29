"""
Communication submodule for agents to have communication. Only implemented for map merging.

__author__ = '{Vinit Jain,Dayyan Smith}'
__project__='{AAIP Project SS19}'
"""

import rospy
import uuid
import numpy as np
from manual_player.msg import Map
from utils import numpy_array_to_csv_string


class Communication():
    def __init__(self, agent_name):
        self._agent_name = agent_name

    def start_map_pub_sub(self, callback, topic_name="map", message_type=Map):
        """
        returns publisher
        """

        rospy.Subscriber(topic_name, message_type, callback)
        pub_map = rospy.Publisher(topic_name, message_type, queue_size=10)

        return pub_map

    def send_map(self, publisher, obstacles, goals, blocks, dispensers, width, height):
        """
        Send map
        """
        msg = Map()
        msg.message_id = self.generate_id()
        msg.agent_name = self._agent_name
        msg.obstacles = numpy_array_to_csv_string(obstacles)
        msg.goals = numpy_array_to_csv_string(goals)
        msg.blocks = numpy_array_to_csv_string(blocks)
        msg.dispensers = numpy_array_to_csv_string(dispensers)
        msg.width = width
        msg.height = height
        publisher.publish(msg)

    def generate_id(self):
        return str(uuid.uuid4())
