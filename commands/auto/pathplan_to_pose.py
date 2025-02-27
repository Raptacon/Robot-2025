# Native imports
from typing import Callable, Tuple

# Internal imports
from config import OperatorRobotConfig

# Third-party imports
from commands2 import Command
from commands2.cmd import none
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathConstraints
from wpimath.geometry import Pose2d
from wpimath.units import degreesToRadians


def pathplanToPose(
    target_pose: Callable[[], Pose2d | None],
    path_constraints: Tuple[float] = OperatorRobotConfig.teleop_pathplan_constraints,
) -> Command:
    """
    THIS CODE IS UNTESTED, PROCEED WITH CAUTION

    Use PathPlanner to navigate from the robot's current position to a given target position
    on the field. PathPlanner uses the navgrid.json file in the deploy folder to determine
    field obstacles - paths generated here will avoid those obstacles.

    Both translational and rotational motion are constrained using a trapezoidal profile,
    where slopes represent max acceleration and the plateau represents max velocity. Other
    configurations (like PID constants) are contained within AutoBuilder and were defined
    when the drivetrain was instantiated.

    Args:
        target_pose: the goal position and orientation on the field in always-blue coordiantes
        path_constraints: the parameters for the trapezoidal motion profile. The first two
            elements define max translational velocity (mps) and acceleration (mps^2),
            the second two elements define max rotational velocity (dps) and acceleration (dps^2).

    Returns:
        If a target pose is available, the command to follow a path to that pose is returned.
            Otherwise an empty command is returned.
    """
    target_pose = target_pose()
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
