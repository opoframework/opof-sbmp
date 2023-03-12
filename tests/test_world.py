import pytest
from ompl_core import solve
from opof_sbmp import World
from opof_sbmp.environments import (
    BookshelfEnvironment,
    CageEnvironment,
    TableEnvironment,
)
from opof_sbmp.robots import BaxterRobot, FetchRobot, UR5Robot


@pytest.mark.parametrize("domain", ["Cage"])
def test_ik_and_plan(domain):

    # Create world.
    world = None
    if domain == "Cage":
        world = World(CageEnvironment(), UR5Robot())
    elif domain == "Bookshelf":
        world = World(BookshelfEnvironment(), FetchRobot())
    elif domain == "Table":
        world = World(TableEnvironment(), BaxterRobot())
    assert world is not None

    # Generate problem
    (scene, problem) = world.rand_problem()
    world.env.set_scene(scene)

    # Solve.
    def validity_checker(x):
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
        None,
        "RRTConnect",
        {},
        problem.start,
        problem.goal,
        5.0,
    )
    assert "success" in result
