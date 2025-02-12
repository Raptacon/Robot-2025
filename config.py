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
        258.92568 / 360.0, 189.14076 / 360.0, 213.2226 / 360.0, 250.83972 / 360.0
    )
    swerve_steer_pid: Tuple[float] = (0.01, 0, 0)
    swerve_drive_pid: Tuple[float] = (0.1, 0, 0.1, 0.23)
    pathplanner_translation_pid: Tuple[float] = (10.0, 0.0, 0.0)
    pathplanner_rotation_pid: Tuple[float] = (5.0, 0.0, 0.0)
