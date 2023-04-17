from time import sleep

from ompl_core import solve
from opof_sbmp import World
from opof_sbmp.domains import SBMPHyp

if __name__ == "__main__":
    domain = SBMPHyp("Bookshelf", "RRTConnect")
    problems = domain.create_problem_set()
    world = World(domain.env_class(), domain.robot_class(), True)

    obj = []
    while True:
        (scene, task) = problems()
        world.env.set_scene(scene)
        world.robot.set_config(task.goal)

        # Projection info. Selects first two joints.
        projection_info = ("Linear", [[0] * len(world.robot.group)] * 2)
        projection_info[1][0][0] = 1
        projection_info[1][1][1] = 1

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
            None,  # projection_info,
            "RRTConnect",
            {"range": 0.7611},
            task.start,
            task.goal,
            3000,
            True,
        )

        for pt in result["trajectory"]:
            world.robot.set_config(pt)
            sleep(0.01)
        for k in result:
            if k != "trajectory":
                print(k, result[k])
