import yaml

from .typing import State


class Task:
    start: State
    goal: State

    def __init__(self, start: State, goal: State):
        self.start = start
        self.goal = goal

    @staticmethod
    def load(path: str) -> "Task":
        with open(path, "r") as f:
            y = yaml.safe_load(f)
        return Task(y["start"], y["goal"])

    def save(self, path: str):
        with open(path, "w") as f:
            yaml.dump({"start": self.start, "goal": self.goal}, f, sort_keys=False)
