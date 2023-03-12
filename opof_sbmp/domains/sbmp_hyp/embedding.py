from typing import List, Tuple

import torch

from ...scene import Scene
from ...task import Task


class SBMPHypEmbedding(torch.nn.Module):
    def __init__(self):
        super(SBMPHypEmbedding, self).__init__()
        self.dummy_param = torch.nn.Parameter(torch.empty(0))

    def forward(self, problem: List[Tuple[Scene, Task]]):
        device = self.dummy_param.device
        dtype = self.dummy_param.dtype
        scene = torch.tensor(
            [x[0].features for x in problem], device=device, dtype=dtype
        )
        start = torch.tensor([x[1].start for x in problem], device=device, dtype=dtype)
        goal = torch.tensor([x[1].goal for x in problem], device=device, dtype=dtype)
        x = torch.concat([scene, start, goal], dim=-1)
        return x
