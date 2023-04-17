from typing import Any

import pkg_resources

from ..robot import Robot


class UR5Robot(Robot):
    @property
    def rrt_range(self):
        return 0.0

    def __init__(self):
        self.setup(
            pkg_resources.resource_filename(
                "opof_sbmp", "descriptions/ur5_description/ur5.urdf"
            ),
            pkg_resources.resource_filename(
                "opof_sbmp", "descriptions/ur5_description/ur5.srdf"
            ),
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
            ["ee_link"],
        )

    def prepare(self, sim: Any):
        sim.setAdditionalSearchPath(
            pkg_resources.resource_filename(
                "opof_sbmp", "descriptions/ur5_description/"
            ),
        )

        # Default initializer.
        Robot.prepare(self, sim)

        sim.resetBasePositionAndOrientation(
            self.robot, [0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]
        )
