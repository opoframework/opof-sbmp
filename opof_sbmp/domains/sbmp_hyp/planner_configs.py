from typing import List, Tuple

PLANNERS = [
    "RRTConnect",
    "LBKPIECE1",
]

TIMEOUTS = {"Cage": 1000, "Bookshelf": 3000, "Table": 5000}

SPACE_HYPERPARAMETERS: List[List[Tuple[str, Tuple[float, float]]]] = [
    [],
    [],
]

PLANNER_HYPERPARAMETERS: List[List[Tuple[str, Tuple[float, float]]]] = [
    [],
    [
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
