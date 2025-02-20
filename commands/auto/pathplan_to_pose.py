# Native imports
from typing import Tuple

# Internal imports
from config import OperatorRobotConfig

# Third-party imports
from commands2 import Command
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathConstraints
from wpimath.geometry import Pose2d
from wpimath.units import degreesToRadians


def pathplanToPose(
    target_pose: Pose2d,
    path_constraints: Tuple[float] = OperatorRobotConfig.teleop_pathplan_constraints,
    rotation_delay_meters: float = 0.0
) -> Command:
    """
    """
    path_constraints = PathConstraints(
        *path_constraints[0:2], degreesToRadians(path_constraints[2]), degreesToRadians(path_constraints[3])
    )

    pathplan_to_pose = AutoBuilder.pathfindToPose(
        target_pose,
        path_constraints,
        goal_end_vel=0.0,
        rotation_delay_distance=rotation_delay_meters
    )

    return pathplan_to_pose
