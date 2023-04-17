from glob import glob
from typing import Any, List, Optional, Tuple

import numpy as np
import pkg_resources
from ompl_core import solve

import opof

from ...environment import Environment
from ...scene import Scene
from ...task import Task
from ...typing import State
from ...world import Robot, World


class SBMPHypPlanner(opof.Planner[Tuple[Scene, Task]]):
    env: Environment
    robot: Robot
    world: World

    planner: str
    timeout: int
    space_hyperparameters: List[Tuple[str, Tuple[float, float]]]
    planner_hyperparameters: List[Tuple[str, Tuple[float, float]]]
    requires_sampler: bool
    requires_projection: bool

    basis: Optional[List[State]]

    def __init__(
        self,
        env: Environment,
        robot: Robot,
        planner: str,
        timeout: int,
        space_hyperparameters: List[Tuple[str, Tuple[float, float]]],
        planner_hyperparameters: List[Tuple[str, Tuple[float, float]]],
        requires_sampler: bool,
        requires_projection: bool,
    ):
        self.env = env
        self.robot = robot
        self.world = World(self.env, self.robot)

        self.planner = planner
        self.timeout = timeout
        self.space_hyperparameters = space_hyperparameters
        self.planner_hyperparameters = planner_hyperparameters
        self.requires_sampler = requires_sampler
        self.requires_projection = requires_projection

        self.basis = []
        env_name = type(self.env).__name__.split("Environment")[0]
        basis_path = pkg_resources.resource_filename(
            "opof_sbmp", f"datasets/{env_name}/basis*.yaml"
        )
        for p in sorted(glob(basis_path))[:50]:
            self.basis.append(self.robot.load_trajectory(p))

    def __call__(
        self,
        problem: Tuple[Scene, Task],
        parameters: List[np.ndarray],
        extras: List[Any],
    ):
        # Set scene in environment.
        self.env.set_scene(problem[0])

        counter = 0

        space_args = []
        planner_args = []
        if len(self.space_hyperparameters) + len(self.planner_hyperparameters) > 0:
            # Extract space args.
            space_args = parameters[counter][: len(self.space_hyperparameters)]
            space_args = [p[0] for p in space_args]
            # Extract planner args.
            planner_args = parameters[counter][len(self.space_hyperparameters) :]
            planner_args = [p[0] for p in planner_args]
            counter += 1
        space_params = dict(
            (
                hp[0],
                hp[1][0] + (hp[1][1] - hp[1][0]) * p,
            )
            for (p, hp) in zip(
                space_args,
                self.space_hyperparameters,
            )
        )
        planner_params = dict(
            (
                hp[0],
                hp[1][0] + (hp[1][1] - hp[1][0]) * p,
            )
            for (p, hp) in zip(
                planner_args,
                self.planner_hyperparameters,
            )
        )
        if self.planner == "RRTConnect":
            planner_params["range"] = self.robot.rrt_range

        # Extract sampler args.
        sampler_args = None
        if self.requires_sampler:
            assert self.basis is not None
            sampler_args = parameters[counter][0]
            # This is necessary since numpy sometimes complains that torch's
            # dirchlet output does not sum to 1.
            sampler_args /= sampler_args.sum()
            counter += 1

        # Extract projection args.
        projection_args = None
        if self.requires_projection:
            projection_args = parameters[counter]
            counter += 1

        assert counter == len(parameters)

        def validity_checker(x: State):
            self.world.robot.set_config(x)
            if not self.robot.check_limits(x):
                return False
            if not self.world.check_collision():
                return False
            return True

        sampler_info = None
        if self.requires_sampler:
            assert sampler_args is not None
            sampler_info = ("PathBasis", self.basis, sampler_args.tolist())

        projection_info = None
        if self.requires_projection:
            assert projection_args is not None
            projection_info = (
                "Linear",
                projection_args.reshape(2, -1).tolist(),
            )

        # Solve.
        result = solve(
            [self.robot.topology[j] for j in self.robot.group],
            space_params,
            sampler_info,
            validity_checker,
            projection_info,
            self.planner,
            planner_params,
            problem[1].start,
            problem[1].goal,
            self.timeout,
            False,
        )

        # Compute objective. We want to minimize time taken, but OPOF
        # maximizes objective. So we use the negative.
        if result["success"] < 0.5:
            result["objective"] = -1.0
        else:
            result["objective"] = -result["iterations"] / self.timeout

        return result
