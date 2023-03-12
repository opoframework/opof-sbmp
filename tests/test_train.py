import pytest
import torch
from opof_sbmp.domains import SBMPHyp

from opof.algorithms import GC

torch.set_num_threads(1)


@pytest.mark.timeout(600)
@pytest.mark.parametrize("domain", ["Cage"])
def test_train_GC(domain):
    domain = SBMPHyp(domain, "RRTConnect")
    algorithm = GC(domain, iterations=10, min_buffer_size=10, eval_interval=5)
    algorithm()
