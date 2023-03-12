from typing import Any, Union

import numpy as np
import yaml

from .. import transformations as t
from ..environment import Environment
from ..scene import Scene
from ..typing import Pose

IK_TOLERANCE = 0.1


def postt(x):
    return np.concatenate((x[1:], x[0:1])).tolist()


def pret(x):
    x = np.array(x)
    return np.concatenate((x[3:], x[0:3]))


class TableScene(Scene):
    table: Pose
    bars_left: Pose
    bars_right: Pose

    def __init__(self, table: Pose, bars_left: Pose, bars_right: Pose):
        self.table = table
        self.bars_left = bars_left
        self.bars_right = bars_right

    @staticmethod
    def load(path: str) -> "TableScene":
        # Extract scene.
        with open(path, "r") as f:
            y = yaml.safe_load(f)
            table = (
                y["table"]["position"],
                y["table"]["orientation"],
            )
            bars_left = (
                y["bars_left"]["position"],
                y["bars_left"]["orientation"],
            )
            bars_right = (
                y["bars_right"]["position"],
                y["bars_right"]["orientation"],
            )
            return TableScene(table, bars_left, bars_right)

    def save(self, path: str):
        with open(path, "w") as f:
            yaml.dump(
                {
                    "table": {
                        "position": self.table[0],
                        "orientation": self.table[1],
                    },
                    "bars_left": {
                        "position": self.bars_left[0],
                        "orientation": self.bars_left[1],
                    },
                    "bars_right": {
                        "position": self.bars_right[0],
                        "orientation": self.bars_right[1],
                    },
                },
                f,
                sort_keys=False,
            )

    @property
    def features(self):
        f = []
        for (pos, rot) in [self.bars_left, self.bars_right]:
            f.append(pos[2])
        return f


class TableEnvironment(Environment[TableScene]):
    table: Any
    bars_left: Any
    bars_right: Any
    sim: Any

    @staticmethod
    def scene_class():
        return TableScene

    def __init__(self):
        self.table = None
        self.bars_left = None
        self.bars_right = None
        self.objects = []
        self.sim = None

    def init(self, sim: Any):
        # Spawn environment
        self.table = TableEnvironment.spawn_object(sim, "table.obj")
        self.bars_left = TableEnvironment.spawn_object(sim, "bars_left.obj")
        self.bars_right = TableEnvironment.spawn_object(sim, "bars_right.obj")
        sim.changeVisualShape(self.bars_left, -1, rgbaColor=[0, 1, 0, 1])
        sim.changeVisualShape(self.bars_right, -1, rgbaColor=[0, 1, 0, 1])
        self.objects = [self.table, self.bars_left, self.bars_right]
        self.sim = sim

    def set_scene(self, scene: TableScene):
        TableEnvironment.set_object_pose(self.sim, self.table, *scene.table)
        TableEnvironment.set_object_pose(self.sim, self.bars_left, *scene.bars_left)
        TableEnvironment.set_object_pose(self.sim, self.bars_right, *scene.bars_right)

    def rand_target(self, scene: TableScene, ee: Union[None, str] = None) -> Pose:
        tf = t.identity_matrix()
        if ee == "left_gripper":
            tf = np.matmul(t.euler_matrix(np.random.uniform(-np.pi, np.pi), 0, 0), tf)
            tf = np.matmul(
                t.translation_matrix(
                    [
                        0.35 + np.random.uniform(-IK_TOLERANCE, IK_TOLERANCE),
                        -0.2 + np.random.uniform(-IK_TOLERANCE, IK_TOLERANCE),
                        0.25 + np.random.uniform(-IK_TOLERANCE, IK_TOLERANCE),
                    ]
                ),
                tf,
            )
        elif ee == "right_gripper":
            tf = np.matmul(t.euler_matrix(np.random.uniform(-np.pi, np.pi), 0, 0), tf)
            tf = np.matmul(
                t.translation_matrix(
                    [
                        0.35 + np.random.uniform(-IK_TOLERANCE, IK_TOLERANCE),
                        0.2 + np.random.uniform(-IK_TOLERANCE, IK_TOLERANCE),
                        0.25 + np.random.uniform(-IK_TOLERANCE, IK_TOLERANCE),
                    ]
                ),
                tf,
            )
        else:
            raise Exception("Unsupported end effector")

        trans = t.translation_from_matrix(tf)
        rot = t.quaternion_from_matrix(t.rotation_matrix(*t.rotation_from_matrix(tf)))

        return (trans, postt(rot))

    def rand_scene(self) -> TableScene:
        return TableScene(
            ([0, 0, 0.0], [0, 0, 0, 1]),
            ([0, np.random.uniform(-0.1, 0.1), 0.0], [0, 0, 0, 1]),
            ([0, np.random.uniform(-0.1, 0.1), 0.0], [0, 0, 0, 1]),
        )

    @staticmethod
    def spawn_object(sim, mesh, position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0]):
        visual = sim.createVisualShape(shapeType=sim.GEOM_MESH, fileName=mesh)
        collision = sim.createCollisionShape(shapeType=sim.GEOM_MESH, fileName=mesh)
        return sim.createMultiBody(
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=position,
            baseOrientation=orientation,
        )

    @staticmethod
    def set_object_pose(sim, o, position, orientation):
        sim.resetBasePositionAndOrientation(o, position, orientation)
