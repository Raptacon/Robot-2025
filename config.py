# Native imports
from typing import Tuple

class OperatorRobotConfig:
    # Default start position for red alliance
    red_default_start_pose: Tuple[float] = (10.0, 1.5, 0.0)
    # Default start position for blue alliance
    blue_default_start_pose: Tuple[float] = (7.7, 6.0, 180.0)
    # Give in front-left, front-right, back-left, back-right order
    # ids for the drive controllers
    swerve_module_channels: Tuple[int] = (50, 53, 56, 59)
    # Give in front-left, front-right, back-left, back-right order
    # starting rotational position for the absolute encoders
    swerve_abs_encoder_calibrations: Tuple[float] = (
        262.96884 / 360.0, 185.0976 / 360.0, 216.91404 / 360.0, 252.24624 / 360.0
    )
    swerve_steer_pid: Tuple[float] = (0.01, 0, 0)
    swerve_drive_pid: Tuple[float] = (0.1, 0, 0.1, 0.23)
    pathplanner_translation_pid: Tuple[float] = (10.0, 0.0, 0.0)
    pathplanner_rotation_pid: Tuple[float] = (5.0, 0.0, 0.0)

    #TODO Get the actual values from our robot
    robot_Cam_Translation: Tuple[float] = (0.5, 0.0, 0.5)
    robot_Cam_Rotation_Degress: Tuple[float] = (0.0, 30.0, 0.0)
    vision_std_dev_basis: float = 1.1
    vision_std_dev_scale_factor: float = 1
    # First three elements are PID, last two elements are trapezoidal profile
    # Translation trapezoidal profile units are mps and mps^2, rotation are dps and dps^2
    pid_to_pose_translation_pid_profile: Tuple[float] = (2.0, 0.0, 0.0, 3, 1.5)
    pid_to_pose_rotation_pid_profile: Tuple[float] = (0.5, 0.0, 0.0, 360, 360)
    # Tolerance of x, y, and omega position errors within which robot is at target pose
    # x error is in meters, y error is in meters, omega error is in degrees
    pid_to_pose_setpoint_tolerances: Tuple[float] = (0.25, 0.25, 15)

    # Robot motion constraints when running PathPlanner during teleop.
    # Values to give are: max translation velocity (mps), max translation acceleration (mps^2),
    # max angular velocity (dps), max angular acceleration (dps^2).
    teleop_pathplan_constraints: Tuple[float] = (3.0, 1.5, 360.0, 360.0)
