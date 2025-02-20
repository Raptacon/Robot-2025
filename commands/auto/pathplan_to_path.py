# Native imports
from typing import Tuple

# Internal imports
from config import OperatorRobotConfig

# Third-party imports
from commands2 import Command
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints
from wpimath.units import degreesToRadians


def pathplanToPath(
    target_path: PathPlannerPath,
    path_constraints: Tuple[float] = OperatorRobotConfig.teleop_pathplan_constraints
) -> Command:
    """
    """
    path_constraints = PathConstraints(
        *path_constraints[0:2], degreesToRadians(path_constraints[2]), degreesToRadians(path_constraints[3])
    )

    pathplan_to_path = AutoBuilder.pathfindThenFollowPath(target_path, path_constraints)

    return pathplan_to_path
