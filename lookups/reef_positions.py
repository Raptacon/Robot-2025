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
    ("red", 0, "l"): Pose2d(14.551, 3.86, Rotation2d.fromDegrees(180)),
    ("red", 0, "r"): Pose2d(14.601, 4.19, Rotation2d.fromDegrees(180)),
    ("red", 1, "l"): Pose2d(13.943, 5.217, Rotation2d.fromDegrees(-120)),
    ("red", 1, "r"): Pose2d(13.683, 5.376, Rotation2d.fromDegrees(-120)),
    ("red", 2, "l"): Pose2d(12.507, 5.356, Rotation2d.fromDegrees(-60)),
    ("red", 2, "r"): Pose2d(12.208, 5.197, Rotation2d.fromDegrees(-60)),
    ("red", 3, "l"): Pose2d(11.599, 4.19, Rotation2d.fromDegrees(0)),
    ("red", 3, "r"): Pose2d(11.599, 3.86, Rotation2d.fromDegrees(0)),
    ("red", 4, "l"): Pose2d(12.188, 2.853, Rotation2d.fromDegrees(60)),
    ("red", 4, "r"): Pose2d(12.477, 2.654, Rotation2d.fromDegrees(60)),
    ("red", 5, "l"): Pose2d(13.683, 2.664, Rotation2d.fromDegrees(120)),
    ("red", 5, "r"): Pose2d(12.963, 2.823, Rotation2d.fromDegrees(120)),
    ("blue", 0, "l"): Pose2d(5.973, 3.86, Rotation2d.fromDegrees(180)),
    ("blue", 0, "r"): Pose2d(6.023, 4.19, Rotation2d.fromDegrees(180)),
    ("blue", 1, "l"): Pose2d(5.365, 5.217, Rotation2d.fromDegrees(-120)),
    ("blue", 1, "r"): Pose2d(5.105, 5.376, Rotation2d.fromDegrees(-120)),
    ("blue", 2, "l"): Pose2d(3.929, 5.356, Rotation2d.fromDegrees(-60)),
    ("blue", 2, "r"): Pose2d(3.63, 5.197, Rotation2d.fromDegrees(-60)),
    ("blue", 3, "l"): Pose2d(3.021, 4.19, Rotation2d.fromDegrees(0)),
    ("blue", 3, "r"): Pose2d(3.021, 3.86, Rotation2d.fromDegrees(0)),
    ("blue", 4, "l"): Pose2d(3.61, 2.853, Rotation2d.fromDegrees(60)),
    ("blue", 4, "r"): Pose2d(3.869, 2.654, Rotation2d.fromDegrees(60)),
    ("blue", 5, "l"): Pose2d(5.105, 2.664, Rotation2d.fromDegrees(120)),
    ("blue", 5, "r"): Pose2d(5.385, 2.823, Rotation2d.fromDegrees(120)),
}

# reef_center_lookup: provides the field coordinate of the center of the reef structure

reef_center_lookup = {
    "red": Translation2d(13.05, 4.025),
    "blue": Translation2d(4.5, 4.025)
}
