from typing import Any

import pkg_resources

from ..robot import Robot


class BaxterRobot(Robot):
    def __init__(self):
        self.setup(
            pkg_resources.resource_filename(
                "opof_sbmp", "descriptions/baxter_description/baxter.urdf"
            ),
            pkg_resources.resource_filename(
                "opof_sbmp", "descriptions/baxter_description/baxter.srdf"
            ),
            [
                "left_s0",
                "left_s1",
                "left_e0",
                "left_e1",
                "left_w0",
                "left_w1",
                "left_w2",
                "right_s0",
                "right_s1",
                "right_e0",
                "right_e1",
                "right_w0",
                "right_w1",
                "right_w2",
            ],
            ["left_gripper", "right_gripper"],
        )

    def prepare(self, sim: Any):
        sim.setAdditionalSearchPath(
            pkg_resources.resource_filename(
                "opof_sbmp", "descriptions/baxter_description/"
            ),
        )

        # Default initializer.
        Robot.prepare(self, sim)

        # Open fingers.
        sim.resetJointState(
            self.robot, self.joint_map["l_gripper_l_finger_joint"], 0.020833
        )
        sim.resetJointState(
            self.robot, self.joint_map["l_gripper_r_finger_joint"], -0.020833
        )
        sim.resetJointState(
            self.robot, self.joint_map["r_gripper_l_finger_joint"], 0.020833
        )
        sim.resetJointState(
            self.robot, self.joint_map["r_gripper_r_finger_joint"], -0.020833
        )

        # Set position.
        sim.resetBasePositionAndOrientation(
            self.robot, [-0.45, 0.0, 0.1], [0.0, 0.0, 0.0, 1.0]
        )
