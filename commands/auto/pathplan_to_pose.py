# Native imports
from typing import Tuple

# Internal imports
from config import OperatorRobotConfig

# Third-party imports
from commands2 import Command
from commands2.cmd import none
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathConstraints
from wpimath.geometry import Pose2d
from wpimath.units import degreesToRadians
from wpilib import SmartDashboard


def pathplanToPose(
    target_pose: Pose2d | None,
    path_constraints: Tuple[float] = OperatorRobotConfig.teleop_pathplan_constraints,
) -> Command:
    """
    """
    if target_pose:
        path_constraints = PathConstraints(
            *path_constraints[0:2], degreesToRadians(path_constraints[2]), degreesToRadians(path_constraints[3])
        )

        pathplan_to_pose = AutoBuilder.pathfindToPose(
            target_pose,
            path_constraints,
            goal_end_vel=0.0
        )

        return pathplan_to_pose

    return none()
