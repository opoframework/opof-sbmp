from .. import transformations as t
from ..environment import Environment
from ..scene import Scene
from ..typing import Pose
from numpy.lib import math
from typing import List, Any, Union
import numpy as np
import yaml


def hom(x):
    return np.concatenate((x, [1]))


def unhom(x):
    return x[:3] / x[3]


def postt(x):
    return np.concatenate((x[1:], x[0:1])).tolist()


def pret(x):
    x = np.array(x)
    return np.concatenate((x[3:], x[0:3]))


def trans(t, x):
    return unhom(np.matmul(t, hom(x)))


class BookshelfScene(Scene):
    bookshelf: Pose
    cans: List[Pose]

    def __init__(self, bookshelf: Pose, cans: List[Pose]):
        self.bookshelf = bookshelf
        self.cans = cans

    @staticmethod
    def load(path: str) -> "BookshelfScene":
        # Extract scene.
        with open(path, "r") as f:
            y = yaml.safe_load(f)
            bookshelf = (
                y["bookshelf"]["position"],
                y["bookshelf"]["orientation"],
            )
            cans = []
            for x in y["cans"]:
                cans.append((x["position"], x["orientation"]))
            return BookshelfScene(bookshelf, cans)

    def save(self, path: str):
        with open(path, "w") as f:
            yaml.dump(
                {
                    "bookshelf": {
                        "position": self.bookshelf[0],
                        "orientation": self.bookshelf[1],
                    },
                    "cans": [
                        {"position": x[0], "orientation": x[1]}
                        for x in self.cans
                    ],
                },
                f,
                sort_keys=False,
            )

    @property
    def features(self):
        f = []
        for (pos, rot) in [self.bookshelf] + self.cans:
            f.extend(pos)
            f.extend(rot)
        return f


class BookshelfEnvironment(Environment[BookshelfScene]):
    bookshelf: Any
    cans: List[Any]
    sim: Any

    @staticmethod
    def scene_class():
        return BookshelfScene

    def __init__(self):
        self.bookshelf = None
        self.cans = []
        self.objects = []
        self.sim = None

    def init(self, sim: Any):
        # Spawn environment
        self.bookshelf = BookshelfEnvironment.spawn_object(
            sim, "bookshelf_tall.obj"
        )
        self.cans = [
            BookshelfEnvironment.spawn_object(sim, "cylinder.obj"),
            BookshelfEnvironment.spawn_object(sim, "cylinder.obj"),
            BookshelfEnvironment.spawn_object(sim, "cylinder.obj"),
            BookshelfEnvironment.spawn_object(sim, "cylinder.obj"),
            BookshelfEnvironment.spawn_object(sim, "cylinder.obj"),
            BookshelfEnvironment.spawn_object(sim, "cylinder.obj"),
            BookshelfEnvironment.spawn_object(sim, "cylinder.obj"),
            BookshelfEnvironment.spawn_object(sim, "cylinder.obj"),
            BookshelfEnvironment.spawn_object(sim, "cylinder.obj"),
        ]
        self.objects = [self.bookshelf] + self.cans
        self.sim = sim

    def set_scene(self, scene: BookshelfScene):
        BookshelfEnvironment.set_object_pose(
            self.sim, self.bookshelf, *scene.bookshelf
        )
        for (i, c) in enumerate(scene.cans):
            BookshelfEnvironment.set_object_pose(self.sim, self.cans[i], *c)

    def rand_target(
        self, scene: BookshelfScene, ee: Union[None, str] = None
    ) -> Pose:
        can = scene.cans[[0, 3, 6][np.random.randint(0, 3)]]
        return (can[0], can[1])

    def rand_scene(self) -> BookshelfScene:
        bookshelf_offset_pos = np.array([np.random.uniform(-0.1, 0.3), 0, 0])
        bookshelf_offset_angle = np.random.uniform(-math.pi / 3, math.pi / 3)
        # DO NOT CHANGE [0.8, 0.0, 0.0]
        bookshelf_pos = np.array([0.8, 0, 0]) + bookshelf_offset_pos
        bookshelf_pos = trans(
            t.euler_matrix(0, 0, bookshelf_offset_angle), bookshelf_pos
        )
        bookshelf = (
            bookshelf_pos.tolist(),
            postt(t.quaternion_from_euler(0, 0, bookshelf_offset_angle)),
        )
        cans = []
        for z in [1.38, 1.08, 0.78]:
            for x in [0.7, 0.5, 0.3]:
                y = np.random.uniform(-0.4, 0.4)
                pos = np.array([x, y, z]) + bookshelf_offset_pos
                pos = trans(t.euler_matrix(0, 0, bookshelf_offset_angle), pos)
                cans.append(
                    (
                        pos.tolist(),
                        postt(
                            t.quaternion_from_euler(
                                0, 0, bookshelf_offset_angle
                            )
                        ),
                    )
                )
        return BookshelfScene(bookshelf, cans)

    @staticmethod
    def spawn_object(
        sim, mesh, position=[0.0, 0.0, 0.0], orientation=[0.0, 0.0, 0.0]
    ):
        visual = sim.createVisualShape(shapeType=sim.GEOM_MESH, fileName=mesh)
        collision = sim.createCollisionShape(
            shapeType=sim.GEOM_MESH, fileName=mesh
        )
        return sim.createMultiBody(
            baseCollisionShapeIndex=collision,
            baseVisualShapeIndex=visual,
            basePosition=position,
            baseOrientation=orientation,
        )

    @staticmethod
    def set_object_pose(sim, o, position, orientation):
        sim.resetBasePositionAndOrientation(o, position, orientation)
