# Native imports
from typing import Tuple

# Third-party imports
from wpimath.geometry import Rotation2d


class OperatorRobotConfig:
    # Default start position for red alliance using always-blue-alliance coordinates
    # Coordinates are x (meters), y (meters), and rotation (degrees)
    red_default_start_pose: Tuple[float] = (10.0, 1.5, 0.0)
    # Default start position for blue alliance using always-blue-alliance coordinates
    # Coordinates are x (meters), y (meters), and rotation (degrees)
    blue_default_start_pose: Tuple[float] = (7.7, 6.0, 180.0)
    # Give in front-left, front-right, back-left, back-right order
    # ids for the drive controllers
    swerve_module_channels: Tuple[int] = (50, 53, 56, 59)
    # Give in front-left, front-right, back-left, back-right order
    # starting rotational position for the absolute encoders
    swerve_abs_encoder_calibrations: Tuple[float] = (
        241.3476 / 360.0, 4.131 / 360.0, 1.14264 / 360.0, 59.41404 / 360.0
    )
    swerve_steer_pid: Tuple[float] = (0.01, 0, 0)
    swerve_drive_pid: Tuple[float] = (0.1, 0, 0.1, 0.23)
    pathplanner_translation_pid: Tuple[float] = (10.0, 0.0, 0.0)
    pathplanner_rotation_pid: Tuple[float] = (5.0, 0.0, 0.0)

    # First three elements are PID, last two elements are trapezoidal profile
    # Translation trapezoidal profile units are mps and mps^2, rotation are rps and rps^2
    pid_to_pose_translation_pid_profile: Tuple[float] = (2.0, 0.0, 0.0, 3, 1.5)
    pid_to_pose_rotation_pid_profile: Tuple[float] = (
        1.0, 0.0, 0.0, Rotation2d.fromDegrees(360).radians(), Rotation2d.fromDegrees(360).radians()
    )
    # Tolerance of x, y, and omega position errors within which robot is at target pose
    # x error is in meters, y error is in meters, omega error is in radians
    pid_to_pose_setpoint_tolerances: Tuple[float] = (0.25, 0.25, Rotation2d.fromDegrees(10).radians())

    # Robot motion constraints when running PathPlanner during teleop.
    # Values to give are: max translation velocity (mps), max translation acceleration (mps^2),
    # max angular velocity (dps), max angular acceleration (dps^2).
    teleop_pathplan_constraints: Tuple[float] = (3.0, 1.5, 360.0, 360.0)
