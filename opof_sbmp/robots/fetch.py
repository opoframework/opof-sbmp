from typing import Any

import pkg_resources

from ..robot import Robot


class FetchRobot(Robot):
    def __init__(self):
        self.setup(
            pkg_resources.resource_filename(
                "opof_sbmp", "descriptions/fetch_description/fetch.urdf"
            ),
            pkg_resources.resource_filename(
                "opof_sbmp", "descriptions/fetch_description/fetch.srdf"
            ),
            [
                "torso_lift_joint",
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "upperarm_roll_joint",
                "elbow_flex_joint",
                "forearm_roll_joint",
                "wrist_flex_joint",
                "wrist_roll_joint",
            ],
            ["gripper_link"],
        )

    def prepare(self, sim: Any):
        sim.setAdditionalSearchPath(
            pkg_resources.resource_filename(
                "opof_sbmp", "descriptions/fetch_description/"
            ),
        )

        # Default initializer.
        Robot.prepare(self, sim)

        # Open fingers.
        for j in ["l_gripper_finger_joint", "r_gripper_finger_joint"]:
            sim.resetJointState(self.robot, self.joint_map[j], 0.05)
