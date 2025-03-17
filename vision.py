from typing import Optional, Tuple

from wpilib import SmartDashboard
from wpimath.geometry import Transform3d, Rotation3d, Translation3d
from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from photonUtils import PhotonUtils
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

from config import OperatorRobotConfig
from lookups.reef_positions import reef_position_lookup
from subsystem.drivetrain.swerve_drivetrain import SwerveDrivetrain


class Vision:
    def __init__(
        self, driveTrain: SwerveDrivetrain,
        left_cam_name: str = "Blue_Port_Left_Side",
        right_cam_name: str = "Side_Port_Right_Side"
    ):
        self.cam_left = PhotonCamera(left_cam_name)
        self.cam_right = PhotonCamera(right_cam_name)
        self.drive = driveTrain
        self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)
        self.reef_tag_ids = {positions["tag"] for positions in reef_position_lookup.values()}

        self.camPoseEstLeft = PhotonPoseEstimator(
            self.field_layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.cam_left,
            Transform3d(
                Translation3d(
                    *OperatorRobotConfig.robot_Cam_Translation_Left
                ),
                Rotation3d.fromDegrees(
                    *OperatorRobotConfig.robot_Cam_Rotation_Degress_Left
                )
            )
        )
        self.camPoseEstRight = PhotonPoseEstimator(
            self.field_layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.cam_right,
            Transform3d(
                Translation3d(
                    *OperatorRobotConfig.robot_Cam_Translation_Right
                ),
                Rotation3d.fromDegrees(
                    *OperatorRobotConfig.robot_Cam_Rotation_Degress_Right
                )
            )
        )

    def getSingleCamEstimate(self, camera: PhotonCamera, poseEstimator: PhotonPoseEstimator) -> None:
        if not ((camera == None) or (poseEstimator == None)):
            unreadPipelines = camera.getAllUnreadResults()
            if len(unreadPipelines) > 0:
                bestPipeline = unreadPipelines[-1]
                camEstPose = poseEstimator.update(bestPipeline)

                if camEstPose:
                    robotPose = camEstPose.estimatedPose.toPose2d()

                    tagDistances = [
                        PhotonUtils.getDistanceToPose(robotPose, self.field_layout.getTagPose(targ.getFiducialId()).toPose2d())
                        for targ in bestPipeline.getTargets()
                        if (targ is not None) and (targ.getPoseAmbiguity() < 0.1) and (targ.getFiducialId() in self.reef_tag_ids)
                    ]

                    distanceToClosestTag = None
                    if len(tagDistances) > 0:
                        distanceToClosestTag = min(tagDistances)

                        stdDev = self.distanceToStdDev(distanceToClosestTag)

                        self.drive.add_vision_pose_estimate(
                            camEstPose.estimatedPose.toPose2d(), camEstPose.timestampSeconds, stdDev
                        )

    def getCamEstimates(self) -> None:
        self.getSingleCamEstimate(self.cam_left, self.camPoseEstLeft)
        self.getSingleCamEstimate(self.cam_right, self.camPoseEstRight)

    def getTargetData(self, target : PhotonTrackedTarget) -> tuple[float, float, float, float]:
        if target == None:
            targetID, targetYaw, targetPitch, targetAmbiguity == (0, 0, 0, 0)
        else:
            targetID = target.getFiducialId()
            targetYaw = target.getYaw()
            targetPitch = target.getPitch()
            targetAmbiguity = target.getPoseAmbiguity()
        return targetID, targetYaw, targetPitch, targetAmbiguity

    def showTargetData(self, target : Optional[PhotonTrackedTarget] = None):
        if target == None:
            target = self.cam_left.getLatestResult().getBestTarget()

            if target == None:
                return

            targetID, targetYaw, targetPitch, targetAmbiguity = self.getTargetData(target)

            SmartDashboard.putNumber("Target ID", targetID)
            SmartDashboard.putNumber("Target Yaw", targetYaw)
            SmartDashboard.putNumber("Target Pitch", targetPitch)
            SmartDashboard.putNumber("Target Ambiguity", targetAmbiguity)

    def distanceToStdDev(self, distance: float | None) -> Tuple[float]:
        std_dev = 3
        if distance:
            if distance > 2:
                # Ignore vision if too far away from tag
                std_dev = 10000
            else:
                std_dev = -1 + (OperatorRobotConfig.vision_std_dev_basis)**(OperatorRobotConfig.vision_std_dev_scale_factor * distance)
        return (std_dev, std_dev, std_dev * 2)
