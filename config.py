# Native imports
from typing import Tuple


class OperatorRobotConfig:
    default_start_pose: Tuple[float] = (2.0, 7.0, 0.0)
    # Give in front-left, front-right, back-left, back-right order
    swerve_module_channels: Tuple[int] = (50, 53, 56, 59)
    swerve_abs_encoder_calibrations: Tuple[float] = (
        258.92568 / 360.0, 189.14076 / 360.0, 213.2226 / 360.0, 250.83972 / 360.0
    )
    swerve_steer_pid: Tuple[float] = (0.01, 0, 0)
    swerve_drive_pid: Tuple[float] = (0.1, 0, 0.1, 1 / 473)
    pathplanner_translation_pid: Tuple[float] = (10.0, 0.0, 0.0)
    pathplanner_rotation_pid: Tuple[float] = (5.0, 0.0, 0.0)
