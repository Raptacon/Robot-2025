# Third-party imports
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

# This uses an always-blue coordinate system
# x and y coordinates are in meters
# rotation is given in degrees

# reef_position_lookup: provides positions the robot should align to on the reef
# to score coral

# structure of the composite key is:
# [0]: alliance the robot is on
# [1]: reef zone the center of the robot is currently in
# [2]: whether to go to the left or right reef pole

reef_position_lookup = {
    ("red", 0, "l"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("red", 0, "r"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("red", 1, "l"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("red", 1, "r"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("red", 2, "l"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("red", 2, "r"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("red", 3, "l"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("red", 3, "r"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("red", 4, "l"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("red", 4, "r"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("red", 5, "l"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("red", 5, "r"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("blue", 0, "l"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("blue", 0, "r"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("blue", 1, "l"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("blue", 1, "r"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("blue", 2, "l"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("blue", 2, "r"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("blue", 3, "l"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("blue", 3, "r"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("blue", 4, "l"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("blue", 4, "r"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("blue", 5, "l"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    ("blue", 5, "r"): Pose2d(0, 0, Rotation2d.fromDegrees(0)),
}

# reef_center_lookup: provides the field coordinate of the center of the reef structure

reef_center_lookup = {
    "red": Translation2d(13.05, 4.025),
    "blue": Translation2d(4.5, 4.025)
}
