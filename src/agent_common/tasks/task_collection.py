"""
Naive strategy
Collects tasks and updates the available tasks for each agent and what task should an agent be assigned to.

__author__ = '{Dayyan Smith, Taha Ahmed}'
__project__='{AAIP Project SS19}'
"""

from task import Task

class TaskCollection():
    def __init__(self):
        self.tasks = {}
        self.assigned_task = None

    def __repr__(self):
        return str(self.tasks.keys())

    def update_tasks(self, tasks_percept, simulation_step):
        '''Update available tasks'''
        new_tasks = self.tasks

        for task in tasks_percept:
            if task.name not in self.tasks.keys():
                new_tasks[task.name] = Task(task_percept=task)

        for task_name, task in new_tasks.iteritems():
            if task.deadline < simulation_step:
                task.expired = True

        self.tasks = new_tasks

    def update_assigned_task(self):
        '''Which tasks should the agent do?'''
        new_assigned_task = None
        try:
            new_assigned_task = next(t for t in self.tasks.values() if not (t.completed or t.expired))
        except StopIteration:
            print("No task to assign")
            pass
        self.assigned_task = new_assigned_task