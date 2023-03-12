from opof_sbmp import SbmpOmplDomain
from opof_sbmp.environments import BookshelfEnvironment
from opof_sbmp.robots import FetchRobot

import numpy as np
import os
from multiprocessing import Queue, Process
from tqdm import tqdm

NUM_WORKERS = 20
NUM_PROBLEMS = 50


def job(worker, queue):
    # Seed RNGs.
    np.random.seed(int.from_bytes(os.urandom(4), byteorder="little"))

    results = []
    domain = SbmpOmplDomain(BookshelfEnvironment, FetchRobot, "RRTConnect")
    training_problems = domain.training_problems()
    planner = domain.planner()
    for i in range(NUM_PROBLEMS):
        index = worker * NUM_PROBLEMS + i
        problem = training_problems()
        queue.put(planner(problem, None))

if __name__ == "__main__":
    queue = Queue()
    for i in range(NUM_WORKERS):
        Process(target=job, args=(i, queue)).start()

    t = 0
    c = 0
    for _ in tqdm(range(NUM_WORKERS * NUM_PROBLEMS), desc="Solving..."):
        r = queue.get()
        t += r[1]
        c += 1

    print(t / c)
