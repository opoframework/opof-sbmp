from typing import Any, List

import numpy as np
import pkg_resources
import pybullet
import pybullet_utils.bullet_client as bc

from .environment import Environment
from .robot import Robot
from .task import Task
from .typing import Pose


class World:
    env: Environment
    robot: Robot

    def __init__(self, env: Environment, robot: Robot, visualize: bool = False):
        self.env = env
        self.robot = robot

        # Create simulator.
        self.sim = bc.BulletClient(
            connection_mode=pybullet.GUI if visualize else pybullet.DIRECT
        )
        self.sim.setAdditionalSearchPath(
            pkg_resources.resource_filename("opof_sbmp", "meshes/"),
        )
        self.sim.setRealTimeSimulation(0)

        # Register to environment
        self.env.init(self.sim)

        # Register robot.
        self.robot.prepare(self.sim)

        # Add collision between robot and environment.
        for i in self.robot.link_map.values():
            for j in self.env.objects:
                self.sim.setCollisionFilterPair(self.robot.robot, j, i, -1, True)

        # Planning details.
        self.dynamic_joints = []
        self.ik_lower: List[Any] = []
        self.ik_upper: List[Any] = []
        self.ik_joint_ranges = []
        for j in range(self.sim.getNumJoints(self.robot.robot)):
            info = self.sim.getJointInfo(self.robot.robot, j)
            if info[2] == pybullet.JOINT_FIXED:
                continue
            self.dynamic_joints.append(info[0])

            limits = self.robot.limits[info[1].decode()]
            if limits is None:
                self.ik_lower.append(0)
                self.ik_upper.append(0)
            else:
                self.ik_lower.append(limits[0])
                self.ik_upper.append(limits[1])
            self.ik_joint_ranges.append(5)

    def check_collision(self):
        self.sim.performCollisionDetection()
        return len(self.sim.getContactPoints()) == 0

    def solve_ik(self, targets: List[Pose], attempts: int):
        for _ in range(attempts):
            chain_configs = []

            # IK per EE chain.
            for (ee_index, ee_link) in enumerate(self.robot.ee_links):
                # Randomize init config.
                self.robot.rand_config()

                # Randomize whatever this does.
                rest_config = []
                for (l, u) in zip(self.ik_lower, self.ik_upper):
                    rest_config.append(np.random.uniform(l, u))

                # Attempt IK.
                ik = self.sim.calculateInverseKinematics(
                    bodyUniqueId=self.robot.robot,
                    endEffectorLinkIndex=self.robot.link_map[ee_link],
                    targetPosition=targets[ee_index][0],
                    targetOrientation=targets[ee_index][1],
                    lowerLimits=self.ik_lower,
                    upperLimits=self.ik_upper,
                    jointRanges=self.ik_joint_ranges,
                    restPoses=rest_config,
                    maxNumIterations=500,
                    jointDamping=[1e-6] * len(self.dynamic_joints),
                    residualThreshold=1e-6,
                )

                chain = []
                chain_current = self.robot.link_map[ee_link]
                while chain_current != -1:
                    (j, n) = self.robot.link_hierarchy[chain_current]
                    chain.append(j)
                    chain_current = n

                # Set chain config.
                chain_config = []
                for (i, x) in enumerate(ik):
                    if (
                        self.robot.joint_map_inv[self.dynamic_joints[i]]
                        in self.robot.group
                        and self.dynamic_joints[i] in chain
                    ):
                        # We need to hold these elsewhere since the states
                        # in sim get overwritten for the next ee chain.
                        chain_config.append((self.dynamic_joints[i], x))
                        chain_configs.append(chain_config)

            # Set config
            for chain_config in chain_configs:
                for (i, x) in chain_config:
                    self.sim.resetJointState(self.robot.robot, i, x)

            # Validate bounds.
            if not self.robot.check_limits(self.robot.get_config()):
                continue

            # Validate tolerance.
            failed_tolerance = False
            for (ee_index, ee_link) in enumerate(self.robot.ee_links):
                # Validate tolerance.
                link_state = self.sim.getLinkState(
                    self.robot.robot,
                    self.robot.link_map[ee_link],
                )
                p_dist = np.max(
                    np.abs(np.array(link_state[0]) - np.array(targets[ee_index][0]))
                )
                q_dist = np.arccos(
                    2
                    * (
                        np.dot(
                            np.array(link_state[1]),
                            np.array(targets[ee_index][1]),
                        )
                        ** 2
                    )
                    - 1
                )
                if p_dist > 0.1 or q_dist > 0.05:
                    failed_tolerance = True
                    break
            if failed_tolerance:
                continue

            # Validate collision.
            if not self.check_collision():
                continue

            return self.robot.get_config()

    def rand_problem(self):
        while True:
            # Sample scene.
            scene = self.env.rand_scene()
            self.env.set_scene(scene)

            # Sample start.
            has_start = False
            for _ in range(100):
                self.robot.rand_config()
                if self.check_collision():
                    has_start = True
                    break
            if not has_start:
                continue
            start = self.robot.get_config()

            # Sample goals.
            ik = self.solve_ik(
                [self.env.rand_target(scene, ee) for ee in self.robot.ee_links],
                100,
            )
            if ik is None:
                continue

            task = Task(start, ik)

            return (scene, task)
