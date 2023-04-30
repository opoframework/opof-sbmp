# opof-sbmp

[OPOF](https://github.com/opoframework/opof) Sampling-based motion planning (SBMP) domains for high-DoF robots to accomplish real-world picking tasks. They include the optimization of planner hyperparameters, sampling distributions, and projections.

[![Build and Test](https://github.com/opoframework/opof-sbmp/actions/workflows/build_and_test.yml/badge.svg)](https://github.com/opoframework/opof-sbmp/actions/workflows/build_and_test.yml)

`opof-sbmp` is maintained by the [Kavraki Lab](https://www.kavrakilab.org/) at Rice University.

### Installation
```console
$ pip install opof-sbmp
```

`opof-sbmp` is officially tested and supported for Python 3.9, 3.10 and Ubuntu 20.04, 22.04.

## Domain: `SBMPHyp[env,planner]`

```python
from opof_sbmp.domains import SBMPHyp
# Creates a SBMPHyp domain instance for the "Bookshelf" environment using the "RRTConnect" planner.
domain = SBMPHyp("Bookshelf", "RRTConnect") 
```

##### Description
We explore doing sampling-based motion planning in a specified [environment](#environments) using a specified [planner](#planners). 
Unlike the grid world domains, these planners are much more complex and the relationship between the choice of planning parameters and planner performance is unclear. 
This makes it particularly suitable for OPOF, since we treat the planner as closed black-box function and specifically assume no knowledge of the planner's internals.

##### Planner optimization problem
The robot is tasked with moving its arm(s) from a start configuration to a goal configuration by running a sampling-based motion planner. 
The planner optimization problem is to find a generator $G_\theta(c)$ that maps a problem instance $c$ 
(in this case, the combination of obstacle poses in the environment and the start and goal robot configurations) to a set of planner hyperparameters 
(which depend on the planner used), such that the number of planner iterations taken for the motion planner to find a path is minimized. 

##### Planning objective
$\boldsymbol{f}(x; c)$ is given as $-iter / {max\\_{iter}}$, where $iter$ is the number of planner iterations taken for the 
motion planner to find a collision-free path from the start to goal robot configuration, and $max\\_iter$ is the maximum allowed
planner iterations.

#### Problem instance distribution
The training set and testing set contain $1000$ and $100$ problem instances respectively. 
These problem instances are adapted from [MotionBenchMaker](https://github.com/KavrakiLab/motion_bench_maker). 
Obstacle positions are sampled according to a predefined distribution, 
while start and goal configurations are sampled using inverse kinematics for some environment-specific task.

## Environments

### `Cage`

<p align="left">
    <img src="https://github.com/opoframework/opof-sbmp/blob/master/docs/_static/cage_goal.png?raw=true" width="350px"/>
</p>

##### Description
A \(6\)-dof UR5 robot is tasked to pick up a block (green) in a cage, starting from a random robot configuration. 
The position and orientation of the cage, as well as the position of the block in the cage, are randomized. 

### `Bookshelf`

<p align="left">
    <img src="https://github.com/opoframework/opof-sbmp/blob/master/docs/_static/bookshelf_goal.png?raw=true" width="350px"/>
</p>

##### Description
A \(8\)-dof Fetch robot is tasked to reach for a cylinder in a bookshelf, starting from a random robot configuration. 
The position and orientation of the bookshelf and cylinders, as well as the choice of cylinder to reach for, are randomized. 

### `Table`

<p align="left">
    <img src="https://github.com/opoframework/opof-sbmp/blob/master/docs/_static/table_goal.png?raw=true" width="350px"/>
</p>

##### Description
A \(14\)-dof dual-arm Baxter robot must fold its arms crossed in a constricted space underneath a table and in between two vertical bars, 
starting from a random robot configuration. The lateral positions of the vertical bars are randomized.

## Planners

### [`RRTConnect`](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1RRTConnect.html#gRRTC)

##### Description
We grow two random search trees from the start and the goal configurations toward randomly sampled target points in the free space, until the two trees connect. 

##### Planner hyperparameters
We tune the following parameters: 
- a _weight_ vector $\in \mathbb{R}^{50}$ with non-negative entries summing to $1$, which controls the sampling of target points using the experience-based sampling scheme adapted from [here](https://ieeexplore.ieee.org/abstract/document/9832486).

The package currently supports simultaneously optimizing other parameters such as the _range_ parameter. However, the learning of such simultaneous parameters
have yet to be properly studied for RRTConnect. Thus, we do not include the these additional parameters by default.

### [`LBKPIECE1`](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1LBKPIECE1.html#gLBKPIECE1)

##### Description
Two random search trees are grown from the start and the goal configurations, but controls the exploration of the configuration space using grid-based projections. 

We note that the learning of conditional hyperparameters for LBKPIECE1 has not been well studied. Thus, this planner should be exploratory for now (i.e. it should 
not be used as a baseline to compare learning algorithms).

##### Planner hyperparameters
We tune the following parameters: 
- _range_ $\in [0.01, 5.00]$ which determines ho wmuch to extend the trees at each step; 
- _border_fraction_ $\in [0.001, 1]$ which determins how much to focus exploration on unexplored cells; 
- _min_valid_path_fraction_ $\in [0.001, 1]$ which determins the threshold for which partially valid extensions are allowed; and
- a _projection_ vector $[0, 1]^{2 \times d} \subset \mathbb{R}^{2 \times d}$ which corresponds to the linear projection function used to induce the 2-dimensional exploration grid, where $d$ is the robot's number of degrees of freedom. 

####

## Citing
If you use `opof-sbmp`, please cite us with:

```
@article{lee23opof,
  author = {Lee, Yiyuan and Lee, Katie and Cai, Panpan and Hsu, David and Kavraki, Lydia E.},
  title = {The Planner Optimization Problem: Formulations and Frameworks},
  booktitle = {arXiv},
  year = {2023},
  doi = {10.48550/ARXIV.2303.06768},
}

@article{lee22apes,
  title={Adaptive Experience Sampling for Motion Planning using the Generator-Critic Framework},
  author = {Lee, Yiyuan and Chamzas, Constantinos and E. Kavraki, Lydia},
  year = {2022},
  month = jul,
  journal={IEEE Robotics and Automation Letters},
}
```

## License

`opof-sbmp` is licensed under the [BSD-3 license](https://github.com/opoframework/opof-sbmp/blob/master/LICENSE.md).

`opof-sbmp` includes a copy of the following libraries as dependencies. These copies are protected and distributed according to the corresponding original license.
- [Boost](https://github.com/opoframework/opof-sbmp/tree/master/ompl_core/boost) ([homepage](https://github.com/boostorg/boost)): [Boost Software License](https://github.com/opoframework/opof-sbmp/tree/master/ompl_core/boost/LICENSE)
- [Eigen](https://github.com/opoframework/opof-sbmp/tree/master/ompl_core/Eigen) ([homepage](https://github.com/bolderflight/eigen)): [MPL2](https://github.com/opoframework/opof-sbmp/tree/master/ompl_core/Eigen/LICENSE)
- [OMPL](https://github.com/opoframework/opof-sbmp/tree/master/ompl_core/ompl) ([homepage](https://github.com/ompl/ompl)): [BSD-3](https://github.com/opoframework/opof-sbmp/tree/master/ompl_core/ompl/LICENSE)

`opof-sbmp` is maintained by the [Kavraki Lab](https://www.kavrakilab.org/) at Rice University, funded in part by NSF RI 2008720 and Rice University funds.
