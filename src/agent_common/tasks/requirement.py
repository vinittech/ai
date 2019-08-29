"""
This contains the requirement class for the block to be needed for an agent.

__author__ = '{Dayyan Smith, Taha Ahmed}'
__project__='{AAIP Project SS19}'
"""

class Requirement:

    def __init__(self, x, y, block_type):
        self.x = x
        self.y = y
        self.block_type = block_type  # 0, 1, 2, ...