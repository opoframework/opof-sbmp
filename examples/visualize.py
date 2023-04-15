from opof_sbmp import World
from opof_sbmp.domains import SBMPHypOpt

if __name__ == "__main__":
    domain = SBMPHypOpt("Bookshelf", "LBKPIECE1")
    problems = domain.create_problem_set()
    p = problems()
    print(len(p[0].features), len(p[1].start), len(p[1].goal))
    for ps in domain.composite_parameter_space():
        print(ps.rand(1).shape)
    world = World(domain.env_class(), domain.robot_class(), True)

    while True:
        (scene, task) = problems()
        world.env.set_scene(scene)
        world.robot.set_config(task.start)
        input()
        world.robot.set_config(task.goal)
        input()
