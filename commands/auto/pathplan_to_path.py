# Native imports
from typing import Callable, Tuple

# Internal imports
from config import OperatorRobotConfig

# Third-party imports
from commands2 import Command
from commands2.cmd import none
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints
from wpimath.units import degreesToRadians


def pathplanToPath(
    target_path: Callable[[], PathPlannerPath | None],
    path_constraints: Tuple[float] = OperatorRobotConfig.teleop_pathplan_constraints
) -> Command:
    """
    Use PathPlanner to navigate from the robot's current position to a given predefined spath.
    PathPlanner uses the navgrid.json file in the deploy folder to determine field obstacles -
    paths generated here will avoid those obstacles. PathPlanner recommends using this approach
    for fine alignment at the end of the complete path.

    Both translational and rotational motion are constrained using a trapezoidal profile,
    where slopes represent max acceleration and the plateau represents max velocity. Other
    configurations (like PID constants) are contained within AutoBuilder and were defined
    when the drivetrain was instantiated.

    Args:
        target_path: the pre-defined path to execute immediately upon completion of the generated
            traversal path
        path_constraints: the parameters for the trapezoidal motion profile. The first two
            elements define max translational velocity (mps) and acceleration (mps^2),
            the second two elements define max rotational velocity (dps) and acceleration (dps^2).

    Returns:
        pathplan_to_path: the command to follow a path that blends into the pre-defined path
    """
    target_path = target_path()
    if target_path:
        path_constraints = PathConstraints(
            *path_constraints[0:2], degreesToRadians(path_constraints[2]), degreesToRadians(path_constraints[3])
        )

        pathplan_to_path = AutoBuilder.pathfindThenFollowPath(target_path, path_constraints)

        return pathplan_to_path

    return none()
