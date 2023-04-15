import argparse
import os
from multiprocessing import Process, Queue

import numpy as np
import opof_sbmp
from ompl_core import solve
from opof_sbmp.environments import (
    BookshelfEnvironment,
    CageEnvironment,
    TableEnvironment,
)
from opof_sbmp.robots import BaxterRobot, FetchRobot, UR5Robot
from opof_sbmp.typing import State
from tqdm import tqdm


def job(config, job_queue, result_queue):
    # Seed RNGs.
    np.random.seed(int.from_bytes(os.urandom(4), byteorder="little"))

    # Create world.
    if config == "Cage":
        world = opof_sbmp.World(CageEnvironment(), UR5Robot())
    elif config == "Bookshelf":
        world = opof_sbmp.World(BookshelfEnvironment(), FetchRobot())
    elif config == "Table":
        world = opof_sbmp.World(TableEnvironment(), BaxterRobot())
    else:
        raise Exception(f"Unsupported config: {config}")

    # Create output folder.
    output = f"../opof_sbmp/datasets/{config}"
    os.makedirs(output, exist_ok=True)

    while True:
        # Generate problem
        (scene, problem) = world.rand_problem()

        # Projection info. Selects first two joints.
        projection_info = ("Linear", [[0] * len(world.robot.group)] * 2)
        projection_info[1][0][0] = 1
        projection_info[1][1][1] = 1

        # Solve.
        def validity_checker(x: State):
            world.robot.set_config(x)
            if not world.robot.check_limits(x):
                return False
            if not world.check_collision():
                return False
            return True

        result = solve(
            [world.robot.topology[j] for j in world.robot.group],
            {},
            None,
            validity_checker,
            None,  # projection_info,
            "RRTConnect",
            {"range": 0.1},
            problem.start,
            problem.goal,
            10000,
            True,
        )
        print(result)

        if result["success"] > 0.5:
            job = job_queue.get()
            # Sentinal value for signalling termination.
            if job is None:
                break
            index = job

            scene.save(f"{output}/scene{index:03d}.yaml")
            problem.save(f"{output}/task{index:03d}.yaml")
            world.robot.write_trajectory(
                result["trajectory"], f"{output}/basis{index:03d}.yaml"
            )
            result_queue.put(index)


if __name__ == "__main__":
    # Arguments.
    parser = argparse.ArgumentParser(description="OPOF SBMP Dataset Generator")
    parser.add_argument(
        "--config",
        type=str,
        choices=["Cage", "Bookshelf", "Table"],
        help="Configuration of problems to solve",
        required=True,
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=1,
        help="Number of workers",
        required=True,
    )
    parser.add_argument(
        "--problems",
        type=int,
        help="Number of problems to generate",
        required=True,
    )
    args = parser.parse_args()

    # Create and run workers.
    job_queue: Queue = Queue()
    result_queue: Queue = Queue()
    for i in range(args.workers):
        Process(
            target=job,
            args=(args.config, job_queue, result_queue),
        ).start()
    for index in range(args.problems):
        job_queue.put(index)
    for i in range(args.workers):
        job_queue.put(None)

    # Wait for complete.
    try:
        for _ in tqdm(range(args.problems), desc="Generating..."):
            result_queue.get()
    finally:
        # Complete.
        for _ in range(args.workers):
            job_queue.put(None)
