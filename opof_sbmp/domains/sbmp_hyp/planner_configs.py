from typing import List, Tuple

PLANNERS = [
    "RRTConnect",
    "LBKPIECE1",
]

TIMEOUTS: List[int] = [3000, 10000]

SPACE_HYPERPARAMETERS: List[List[Tuple[str, Tuple[float, float]]]] = [
    [],
    [],
]

PLANNER_HYPERPARAMETERS: List[List[Tuple[str, Tuple[float, float]]]] = [
    [("range", (0.01, 5.00))],
    [
        ("range", (0.01, 5.00)),
        ("border_fraction", (0.001, 1.000)),
        ("min_valid_path_fraction", (0.001, 1.000)),
    ],
]

REQUIRES_SAMPLER = [
    True,
    False,
]

REQUIRES_PROJECTION = [
    False,
    True,
]
