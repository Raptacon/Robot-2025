# Native imports
from typing import Callable, Dict

# Internal imports
from lookups.reef_positions import reef_center_lookup

# Third-party imports
from wpimath.geometry import Pose2d, Rotation2d, Translation2d


def getCurrentReefZone(
    alliance: str,
    current_pose: Callable[[], Pose2d],
    reef_center_lookup: Dict[str, Translation2d] = reef_center_lookup
) -> int:
    """
    Get the positional zone of the robot relative to the reef. Zones represent field coordinate
    sections defined using lines projecting from the center of the reef through the corners
    of the hexagon. Zones adhere to blue alliance field coordinates and CCW positive conventions;
    so, the rightmost hexagon face is zone 0 on either alliance. Zones thereafter increment by 1
    in a CCW fashion.

    Instead of using zone polygons, we instead define an angle between the robot-to-reef-center
    vector and a centralized x-axis and coerce that angle into zone integers. Because the rightmost
    part of the hexagon is a face instead of a corner, we define and apply an offset equal to half a hexagon
    corner angle.

    Args:
        alliance: which alliance the robot is on. Enum["red", "blue"]
        current_pose: the current blue-alliance field coordinate and orientation
        reef_center_lookup: a map between alliance names and that alliance's reef centerpoint

    Returns:
        current_reef_zone: the alliance reef zone the robot is currently in. Domain of [0, 5]
    """
    zero_offset = Rotation2d.fromDegrees(-360.0 / 12.0)
    reef_center = reef_center_lookup[alliance]
    angle_from_center = (current_pose().translation() - reef_center).angle()
    angle_from_center = angle_from_center - zero_offset

    current_reef_zone = int((angle_from_center.degrees() % 360.0) // 60)
    return current_reef_zone
