from abc import abstractmethod
from typing import Any, Dict, List, Set, Tuple, Union

import numpy as np
import pybullet
import yaml
from urdf_parser_py import urdf

from .srdf_parser import SRDF
from .typing import Group, Pose, State, Topology


class Robot(object):
    urdf_path: str
    srdf_path: str
    urdf_model: Any
    collisions: Set[Tuple[str, str]]
    topology: Topology
    group: Group
    ee_links: List[str]
    limits: Dict[str, Union[Tuple[float, float], None]]

    sim: Any
    joint_map: Dict[str, int]
    joint_map_inv: Dict[int, str]
    link_map: Dict[str, int]
    link_map_inv: Dict[int, str]
    link_hierarchy: Dict[int, Tuple[int, int]]
    robot: Any

    def __init__(self):
        raise NotImplementedError()

    @property
    @abstractmethod
    def rrt_range(self):
        raise NotImplementedError()

    def setup(self, urdf_path: str, srdf_path: str, group: Group, ee_links: List[str]):
        self.urdf_path = urdf_path
        self.srdf_path = srdf_path
        self.group = group
        self.ee_links = ee_links

        # Load robot URDF and SRDF.
        with open(urdf_path) as f:
            self.urdf_model = urdf.Robot.from_xml_string(f.read())
        with open(srdf_path) as f:
            self.disable_collisions = set()
            srdf = SRDF.from_xml_string(f.read())
            for dc in srdf.disable_collisionss:
                self.disable_collisions.add((dc.link1, dc.link2))

        # Compute robot topology.
        self.topology = dict()
        self.limits = dict()
        for joint in self.urdf_model.joints:
            # Topology.
            if joint.dynamics is None:
                self.topology[joint.name] = None
                self.limits[joint.name] = None
            else:
                if joint.type == "prismatic":
                    if joint.limit is None or joint.limit.lower == joint.limit.upper:
                        raise Exception("Prismatic joint must be bounded")

                    self.topology[joint.name] = (
                        "P",
                        joint.limit.lower,
                        joint.limit.upper,
                    )
                    self.limits[joint.name] = (
                        joint.limit.lower,
                        joint.limit.upper,
                    )
                elif joint.type == "revolute" or "continuous":
                    self.topology[joint.name] = ("R",)
                    if joint.limit is None or joint.limit.lower == joint.limit.upper:
                        self.limits[joint.name] = (-np.pi, np.pi)
                    else:
                        self.limits[joint.name] = (
                            joint.limit.lower,
                            joint.limit.upper,
                        )
                else:
                    raise Exception("Unsupported topology")

    def check_limits(self, x: State) -> bool:
        for (j, v) in zip(self.group, x):
            if self.limits[j] is not None:
                if v < self.limits[j][0]:
                    return False
                if v > self.limits[j][1]:
                    return False
        return True

    def prepare(self, sim: Any):
        self.sim = sim
        self.robot = sim.loadURDF(
            self.urdf_path,
            useFixedBase=True,
            flags=pybullet.URDF_USE_SELF_COLLISION
            | pybullet.URDF_USE_SELF_COLLISION_INCLUDE_PARENT,
        )

        # Joint to index map.
        self.joint_map = dict()
        self.joint_map_inv = dict()
        self.link_map = dict()
        self.link_map_inv = dict()
        self.link_map[self.sim.getBodyInfo(self.robot)[0].decode()] = -1
        # Compute joints and links.
        for j in range(self.sim.getNumJoints(self.robot)):
            info = self.sim.getJointInfo(self.robot, j)
            self.joint_map[info[1].decode()] = info[0]
            self.link_map[info[12].decode()] = info[0]
        for k in self.joint_map:
            self.joint_map_inv[self.joint_map[k]] = k
        for k in self.link_map:
            self.link_map_inv[self.link_map[k]] = k
        # Compute hierarchy.
        self.link_hierarchy = dict()
        for j in range(self.sim.getNumJoints(self.robot)):
            info = self.sim.getJointInfo(self.robot, j)
            child_link = j
            joint = info[0]
            parent_link = info[16]
            self.link_hierarchy[child_link] = (joint, parent_link)

        # Set collisions.
        for a in self.link_map.keys():
            for b in self.link_map.keys():
                disable = (
                    a == b
                    or ((a, b) in self.disable_collisions)
                    or ((b, a) in self.disable_collisions)
                )
                self.sim.setCollisionFilterPair(
                    self.robot,
                    self.robot,
                    self.link_map[a],
                    self.link_map[b],
                    int(not disable),
                )

    def set_config(self, config: List[float]):
        for (i, j) in enumerate(self.group):
            self.sim.resetJointState(self.robot, self.joint_map[j], config[i])

    def rand_config(self):
        # Randomize init config.
        for (i, j) in enumerate(self.group):
            limit = self.limits[j] if j in self.limits else [0, 0]
            self.sim.resetJointState(
                self.robot,
                self.joint_map[j],
                np.random.uniform(limit[0], limit[1]),
            )

    def get_config(self, joint: Union[None, str] = None):
        if joint is None:
            return [
                self.sim.getJointState(self.robot, self.joint_map[j])[0]
                for j in self.group
            ]
        else:
            return self.sim.getJointState(self.robot, self.joint_map[joint])[0]

    def get_pose(self, link: str) -> Pose:
        return self.sim.getLinkState(self.robot, self.link_map[link])[4:6]

    def write_trajectory(self, trajectory: List[State], path: str):
        with open(path, "w") as f:
            yaml.dump(trajectory, f)

    def load_trajectory(self, path: str):
        with open(path, "r") as f:
            return yaml.safe_load(f)
