from typing import Generic, List, Tuple, Type, TypeVar

import pkg_resources

import opof
from opof.evaluators import ListEvaluator
from opof.parameter_spaces import Interval, Simplex
from opof.problem_sets import ProblemList

from ...environment import Environment
from ...environments import BookshelfEnvironment, CageEnvironment, TableEnvironment
from ...robots import BaxterRobot, FetchRobot, UR5Robot
from ...scene import Scene
from ...task import Task
from ...world import Robot
from .embedding import SBMPHypEmbedding
from .planner import SBMPHypPlanner
from .planner_configs import (
    PLANNER_HYPERPARAMETERS,
    PLANNERS,
    REQUIRES_PROJECTION,
    REQUIRES_SAMPLER,
    SPACE_HYPERPARAMETERS,
    TIMEOUTS,
)

TEnvironment = TypeVar("TEnvironment", bound=Environment)
TRobot = TypeVar("TRobot", bound=Robot)


class SBMPHyp(opof.Domain[Tuple[Scene, Task]], Generic[TEnvironment, TRobot]):
    env_class: Type[TEnvironment]
    robot_class: Type[TRobot]
    dataset_path: str
    planner: str
    projection: str

    timeout: int
    planner_hyperparameters: List[Tuple[str, Tuple[float, float]]]
    space_hyperparameters: List[Tuple[str, Tuple[float, float]]]
    requires_sampler: bool
    requires_projection: bool

    def __init__(
        self,
        env_name: str,
        planner: str,
    ):
        self.env_class = {
            "Cage": CageEnvironment,
            "Bookshelf": BookshelfEnvironment,
            "Table": TableEnvironment,
        }[env_name]
        self.robot_class = {
            "Cage": UR5Robot,
            "Bookshelf": FetchRobot,
            "Table": BaxterRobot,
        }[env_name]
        self.dataset_path = pkg_resources.resource_filename(
            "opof_sbmp", f"datasets/{env_name}"
        )

        self.planner = planner

        self.timeout = TIMEOUTS[env_name]
        self.space_hyperparameters = SPACE_HYPERPARAMETERS[PLANNERS.index(planner)]
        self.planner_hyperparameters = PLANNER_HYPERPARAMETERS[PLANNERS.index(planner)]
        self.requires_sampler = REQUIRES_SAMPLER[PLANNERS.index(planner)]
        self.requires_projection = REQUIRES_PROJECTION[PLANNERS.index(planner)]

    def create_planner(self):
        return SBMPHypPlanner(
            self.env_class(),
            self.robot_class(),
            self.planner,
            self.timeout,
            self.space_hyperparameters,
            self.planner_hyperparameters,
            self.requires_sampler,
            self.requires_projection,
        )

    def create_problem_set(self):
        train_problems = []
        for i in range(900):
            scene = self.env_class.scene_class().load(
                f"{self.dataset_path}/scene{i:03d}.yaml"
            )
            task = Task.load(f"{self.dataset_path}/task{i:03d}.yaml")
            train_problems.append((scene, task))
        return ProblemList(train_problems)

    def composite_parameter_space(self):
        spaces = []
        # State space parameters.
        if len(self.space_hyperparameters) + len(self.planner_hyperparameters) > 0:
            spaces.append(
                Interval(
                    len(self.space_hyperparameters) + len(self.planner_hyperparameters)
                )
            )
        # Sampler parameters.
        if self.requires_sampler:
            spaces.append(Simplex(1, 50))
        # Projection parameters.
        if self.requires_projection:
            spaces.append(Interval(2 * len(self.robot_class().group)))
        return spaces

    def create_problem_embedding(self):
        return SBMPHypEmbedding()

    def create_evaluator(self):
        eval_problems = []
        for i in range(900, 1000):
            scene = self.env_class.scene_class().load(
                f"{self.dataset_path}/scene{i:03d}.yaml"
            )
            task = Task.load(f"{self.dataset_path}/task{i:03d}.yaml")
            eval_problems.append((scene, task))
        return ListEvaluator(self, eval_problems)
