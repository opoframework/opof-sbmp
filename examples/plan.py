import os
from multiprocessing import Process, Queue

import numpy as np
from tqdm import tqdm
from opof_sbmp.domains import SBMPHyp

NUM_WORKERS = 20
NUM_PROBLEMS = 50


def job(worker, queue):
    # Seed RNGs.
    np.random.seed(int.from_bytes(os.urandom(4), byteorder="little"))

    domain = SBMPHyp("Bookshelf", "RRTConnect")
    problems = domain.create_problem_set()
    planner = domain.create_planner()
    for i in range(NUM_PROBLEMS):
        index = worker * NUM_PROBLEMS + i
        problem = problems()
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
