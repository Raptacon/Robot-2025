# Third-party imports
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d

# This uses an always-blue coordinate system
# x and y coordinates are in meters
# rotation is given in degrees

# reef_position_lookup: provides positions the robot should align to on the reef
# to score coral. The initial given poses are the true poses of the reef scoring locations.
# These are then adjusted by the given robot-relative offset to get the final scoring poses.

# structure of the composite key is:
# [0]: alliance the robot is on
# [1]: reef zone the center of the robot is currently in
# [2]: whether to go to the left or right reef pole

robot_relative_offset = (Translation2d(-0.438, 0.127), Rotation2d.fromDegrees(0))

reef_position_lookup = {
    ("red", 0, "l"): {"pose": Pose2d(13.8905257810516, 3.8615700518161, Rotation2d.fromDegrees(180)), "path": "Stem_Reef_N1", "tag": 7},
    ("red", 0, "r"): {"pose": Pose2d(13.8905257810516, 4.1902460518161, Rotation2d.fromDegrees(180)), "path": "Stem_Reef_F1", "tag": 7},
    ("red", 1, "l"): {"pose": Pose2d(13.616793831753, 4.66332249098298, Rotation2d.fromDegrees(240)), "path": "Stem_Reef_F2", "tag": 8},
    ("red", 1, "r"): {"pose": Pose2d(13.3321520661388, 4.82766049098298, Rotation2d.fromDegrees(240)), "path": "Stem_Reef_F3", "tag": 8},
    ("red", 2, "l"): {"pose": Pose2d(12.7857041695737, 4.82766049098298, Rotation2d.fromDegrees(300)), "path": "Stem_Reef_F4", "tag": 9},
    ("red", 2, "r"): {"pose": Pose2d(12.5010624039595, 4.66332249098298, Rotation2d.fromDegrees(300)), "path": "Stem_Reef_F5", "tag": 9},
    ("red", 3, "l"): {"pose": Pose2d(12.2273304546609, 4.1902460518161, Rotation2d.fromDegrees(0)), "path": "Stem_Reef_F6", "tag": 10},
    ("red", 3, "r"): {"pose": Pose2d(12.2273304546609, 3.8615700518161, Rotation2d.fromDegrees(0)), "path": "Stem_Reef_N6", "tag": 10},
    ("red", 4, "l"): {"pose": Pose2d(12.5010624039595, 3.38849361264923, Rotation2d.fromDegrees(60)), "path": "Stem_Reef_N5", "tag": 11},
    ("red", 4, "r"): {"pose": Pose2d(12.7857041695737, 3.22415561264923, Rotation2d.fromDegrees(60)), "path": "Stem_Reef_N4", "tag": 11},
    ("red", 5, "l"): {"pose": Pose2d(13.3321520661388, 3.22415561264923, Rotation2d.fromDegrees(120)), "path": "Stem_Reef_N3", "tag": 6},
    ("red", 5, "r"): {"pose": Pose2d(13.616793831753, 3.38849361264923, Rotation2d.fromDegrees(120)), "path": "Stem_Reef_N2", "tag": 6},
    ("blue", 0, "l"): {"pose": Pose2d(5.32105664211329, 3.8615700518161, Rotation2d.fromDegrees(180)), "path": "Stem_Reef_F6", "tag": 21},
    ("blue", 0, "r"): {"pose": Pose2d(5.32105664211329, 4.1902460518161, Rotation2d.fromDegrees(180)), "path": "Stem_Reef_N6", "tag": 21},
    ("blue", 1, "l"): {"pose": Pose2d(5.04707069230675, 4.66332249098298, Rotation2d.fromDegrees(240)), "path": "Stem_Reef_N5", "tag": 20},
    ("blue", 1, "r"): {"pose": Pose2d(4.76242892669249, 4.82766049098298, Rotation2d.fromDegrees(240)), "path": "Stem_Reef_N4", "tag": 20},
    ("blue", 2, "l"): {"pose": Pose2d(4.21623503063543, 4.82766049098298, Rotation2d.fromDegrees(300)), "path": "Stem_Reef_N3", "tag": 19},
    ("blue", 2, "r"): {"pose": Pose2d(3.93159326502117, 4.66332249098298, Rotation2d.fromDegrees(300)), "path": "Stem_Reef_N2", "tag": 19},
    ("blue", 3, "l"): {"pose": Pose2d(3.65760731521463, 4.1902460518161, Rotation2d.fromDegrees(0)), "path": "Stem_Reef_N1", "tag": 18},
    ("blue", 3, "r"): {"pose": Pose2d(3.65760731521463, 3.8615700518161, Rotation2d.fromDegrees(0)), "path": "Stem_Reef_F1", "tag": 18},
    ("blue", 4, "l"): {"pose": Pose2d(3.93159326502117, 3.38849361264923, Rotation2d.fromDegrees(60)), "path": "Stem_Reef_F2", "tag": 17},
    ("blue", 4, "r"): {"pose": Pose2d(4.21623503063543, 3.22415561264923, Rotation2d.fromDegrees(60)), "path": "Stem_Reef_F3", "tag": 17},
    ("blue", 5, "l"): {"pose": Pose2d(4.76242892669249, 3.22415561264923, Rotation2d.fromDegrees(120)), "path": "Stem_Reef_F4", "tag": 22},
    ("blue", 5, "r"): {"pose": Pose2d(5.04707069230675, 3.38849361264923, Rotation2d.fromDegrees(120)), "path": "Stem_Reef_F5", "tag": 22},
}

for key in reef_position_lookup:
    reef_position_lookup[key]["pose"] = reef_position_lookup[key]["pose"].transformBy(Transform2d(*robot_relative_offset))

# reef_center_lookup: provides the field coordinate of the center of the reef structure in meters

reef_center_lookup = {
    "red": Translation2d(13.05, 4.025),
    "blue": Translation2d(4.5, 4.025)
}

# reef_height_lookup: provides the height, in centimeters, of a reef scoring level

reef_height_lookup = {
    "L1": 46,
    "L2": 81,
    "L3": 121,
    "L4": 183
}
