import sys

import torch
from opof_sbmp.domains import SBMPHyp

from opof.models import FCResNetGenerator

if __name__ == "__main__":
    domain = SBMPHyp("Bookshelf", "RRTConnect")
    problems = domain.create_problem_set()
    planner = domain.create_planner()
    generator = FCResNetGenerator(domain)
    generator.load_state_dict(torch.load(sys.argv[1]))
    generator.eval()

    while True:
        problem = problems()
        parameters = generator([problem])
        print(parameters)

