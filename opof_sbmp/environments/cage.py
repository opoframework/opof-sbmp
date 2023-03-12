from typing import Any, Union

import numpy as np
import yaml
from numpy.lib import math

from .. import transformations as t
from ..environment import Environment
from ..scene import Scene
from ..typing import Pose


def postt(x):
    return np.concatenate((x[1:], x[0:1])).tolist()


def pret(x):
    x = np.array(x)
    return np.concatenate((x[3:], x[0:3]))


class CageScene(Scene):
    cage: Pose
    cube: Pose

    def __init__(self, cage: Pose, cube: Pose):
        self.cage = cage
        self.cube = cube

    @staticmethod
    def load(path: str) -> "CageScene":
        # Extract scene.
        with open(path, "r") as f:
            y = yaml.safe_load(f)
            cage = (
                y["cage"]["position"],
                y["cage"]["orientation"],
            )
            cube = (
                y["cube"]["position"],
                y["cube"]["orientation"],
            )
            return CageScene(cage, cube)

    def save(self, path: str):
        with open(path, "w") as f:
            yaml.dump(
                {
                    "cage": {
                        "position": self.cage[0],
                        "orientation": self.cage[1],
                    },
                    "cube": {
                        "position": self.cube[0],
                        "orientation": self.cube[1],
                    },
                },
                f,
                sort_keys=False,
            )

    @property
    def features(self):
        f = []
        for (pos, rot) in [self.cage, self.cube]:
            f.extend(pos)
            f.extend(rot)
        return f


class CageEnvironment(Environment[CageScene]):
    cage: Any
    cube: Any
    sim: Any

    @staticmethod
    def scene_class():
        return CageScene

    def __init__(self):
        self.cage = None
        self.cube = None
        self.objects = []
        self.sim = None

    def init(self, sim: Any):
        # Spawn environment
        self.cage = CageEnvironment.spawn_object(sim, "cage.obj")
        self.cube = CageEnvironment.spawn_object(sim, "cube.obj")
        sim.changeVisualShape(self.cube, -1, rgbaColor=[0, 1, 0, 1])
        self.objects = [self.cage, self.cube]
        self.sim = sim

    def set_scene(self, scene: CageScene):
        CageEnvironment.set_object_pose(self.sim, self.cage, *scene.cage)
        CageEnvironment.set_object_pose(self.sim, self.cube, *scene.cube)

    def rand_target(self, scene: CageScene, ee: Union[None, str] = None) -> Pose:
        tf = t.identity_matrix()
        tf = np.matmul(t.translation_matrix([-0.2, 0.0, 0.0]), tf)
        tf = np.matmul(t.euler_matrix(0, math.pi / 2, 0), tf)
        tf = np.matmul(t.quaternion_matrix(pret(scene.cube[1])), tf)
        tf = np.matmul(t.translation_matrix(scene.cube[0]), tf)

        trans = t.translation_from_matrix(tf)
        rot = t.quaternion_from_matrix(t.rotation_matrix(*t.rotation_from_matrix(tf)))

        return (trans, postt(rot))

    def rand_scene(self) -> CageScene:
        tf = t.identity_matrix()
        tf = np.matmul(t.translation_matrix([np.random.uniform(0.5, 0.9), 0, 0]), tf)
        tf = np.matmul(
            t.euler_matrix(0, 0, np.random.uniform(-math.pi / 3, math.pi / 3)),
            tf,
        )

        cage = (
            t.translation_from_matrix(tf).tolist(),
            postt(t.quaternion_from_matrix(tf)),
        )

        cube = np.matmul(
            tf,
            t.translation_matrix(
                [
                    np.random.uniform(-0.1, 0.1),
                    np.random.uniform(-0.1, 0.1),
                    0.0825,
                ]
            ),
        )
        cube = (
            t.translation_from_matrix(cube).tolist(),
            postt(t.quaternion_from_matrix(cube)),
        )

        return CageScene(cage, cube)

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
