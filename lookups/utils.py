# Native imports
from typing import Callable

# Internal imports
from lookups.reef_positions import reef_center_lookup

# Third-party imports
from wpimath.geometry import Pose2d, Rotation2d


def getCurrentReefZone(alliance: str, current_pose: Callable[[], Pose2d], reef_center_lookup: dict = reef_center_lookup) -> int:
    """
    """
    zero_offset = Rotation2d.fromDegrees(-360.0 / 12.0)
    reef_center = reef_center_lookup[alliance]
    angle_from_center = (current_pose().translation() - reef_center).angle()
    angle_from_center = angle_from_center - zero_offset

    current_reef_zone = int((angle_from_center.degrees() % 360.0) // 60)
    return current_reef_zone
