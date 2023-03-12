from typing import Dict, List, Tuple, Union

Dimension = Union[Tuple[str, float, float], Tuple[str]]
Topology = Dict[str, Union[Dimension, None]]
Group = List[str]
State = List[float]
Position = List[float]
Orientation = List[float]
Pose = Tuple[Position, Orientation]
