from typing import List, Tuple

TIMEOUT = {"Cage": 1.0, "Bookshelf": 2.0, "Table": 3.0}

PLANNERS = [
    "RRTConnect",
    "BiEST",
    "LBKPIECE1",
]

SPACE_HYPERPARAMETERS: List[List[Tuple[str, Tuple[float, float]]]] = [
    [],
    [],
    [],
]

PLANNER_HYPERPARAMETERS: List[List[Tuple[str, Tuple[float, float]]]] = [
    [("range", (0.01, 5.00))],
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
    False,
]

REQUIRES_PROJECTION = [
    False,
    False,
    True,
]
