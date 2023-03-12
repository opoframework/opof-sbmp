from typing import Any, Generic, List, TypeVar, Union

from .scene import Scene
from .typing import Pose

TScene = TypeVar("TScene", bound=Scene)


class Environment(Generic[TScene]):
    objects: List[Any]

    @staticmethod
    def scene_class() -> TScene:
        raise NotImplementedError()

    def init(self, sim: Any):
        raise NotImplementedError()

    def set_scene(self, scene: TScene):
        raise NotImplementedError()

    def rand_target(self, scene: TScene, ee: Union[None, str] = None) -> Pose:
        raise NotImplementedError()

    def rand_scene(self) -> TScene:
        raise NotImplementedError()
