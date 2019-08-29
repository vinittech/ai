"""
Naive task allocation class for an agent.

__author__ = '{Dayyan Smith,Taha Ahmed}'
__project__='{AAIP Project SS19}'

"""

from requirement import Requirement


class Task:
    def __init__(self, task_percept):
        self.name = task_percept.name
        self.deadline = task_percept.deadline
        self.expired = False
        self.completed = False

        # Assume tasks only have one required block
        x = task_percept.requirements[0].pos.x
        y = task_percept.requirements[0].pos.y

        # Remove 'b'
        block_type = int(task_percept.requirements[0].type[1:])

        self.requirement = Requirement(x, y, block_type)

    def __repr__(self):
        return self.name
