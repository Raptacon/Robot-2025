from typing import Optional, Tuple

from wpilib import SmartDashboard
from wpimath.geometry import Transform3d, Rotation3d, Translation3d
from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from photonUtils import PhotonUtils
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

from subsystem.drivetrain.swerve_drivetrain import SwerveDrivetrain
from config import OperatorRobotConfig

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
            self.cam_left,
            Transform3d(
                Translation3d(
                    *OperatorRobotConfig.robot_Cam_Translation_Right
                ),
                Rotation3d.fromDegrees(
                    *OperatorRobotConfig.robot_Cam_Rotation_Degress_Right
                )
            )
        )


    def getCamEstimate(self):
        bestPipelineLeft = self.cam_left.getLatestResult()
        camEstPoseLeft = self.camPoseEstLeft.update(bestPipelineLeft)
        if camEstPoseLeft:
            robot_pose = camEstPoseLeft.estimatedPose.toPose2d()

            tag_distances = [
                PhotonUtils.getDistanceToPose(robot_pose, self.field_layout.getTagPose(targ.getFiducialId()).toPose2d())
                for targ in bestPipelineLeft.getTargets()
                if targ is not None
            ]

            distance_to_closest_tag = None
            if len(tag_distances) > 0:
                distance_to_closest_tag = min(tag_distances)

            std_dev = self.distanceToStdDev(distance_to_closest_tag)

            self.drive.add_vision_pose_estimate(
                camEstPoseLeft.estimatedPose.toPose2d(), camEstPoseLeft.timestampSeconds, std_dev
            )

        bestPipelineRight = self.cam_right.getLatestResult()
        camEstPoseRight = self.camPoseEstRight.update(bestPipelineRight)
        if camEstPoseRight:
            robot_pose = camEstPoseRight.estimatedPose.toPose2d()
            
            tag_distances = [
                PhotonUtils.getDistanceToPose(robot_pose, self.field_layout.getTagPose(targ.getFiducialId()).toPose2d())
                for targ in bestPipelineRight.getTargets()
                if targ is not None
            ]

            distance_to_closest_tag = None
            if len(tag_distances) > 0:
                distance_to_closest_tag = min(tag_distances)

            std_dev = self.distanceToStdDev(distance_to_closest_tag)

            self.drive.add_vision_pose_estimate(
                camEstPoseRight.estimatedPose.toPose2d(), camEstPoseRight.timestampSeconds, std_dev
            )

    def getTargetData(self, target : PhotonTrackedTarget) -> tuple[float, float, float, float]:
        targetID = target.getFiducialId()
        targetYaw = target.getYaw()
        targetPitch = target.getPitch()
        targetAmbiguity = target.getPoseAmbiguity()
        return targetID, targetYaw, targetPitch, targetAmbiguity
    
    def showTargetData(self, target : Optional[PhotonTrackedTarget] = None):
        if target == None:
            target = self.cam_left.getLatestResult().getBestTarget()

        if(not self.cam_left.getLatestResult().hasTargets() or self.cam_left.getLatestResult() == None):
            return

        targetID, targetYaw, targetPitch, targetAmbiguity = self.getTargetData(target)

        SmartDashboard.putNumber("Target ID", targetID)
        SmartDashboard.putNumber("Target Yaw", targetYaw)
        SmartDashboard.putNumber("Target Pitch", targetPitch)
        SmartDashboard.putNumber("Target Ambiguity", targetAmbiguity)

    def distanceToStdDev(self, distance: float | None) -> Tuple[float]:
        std_dev = 3
        SmartDashboard.putNumber("vis dist", distance)
        if distance:
            if distance > 2:
                # Ignore vision if too far away from tag
                std_dev = 10000
            else:
                std_dev = -1 + (OperatorRobotConfig.vision_std_dev_basis)**(OperatorRobotConfig.vision_std_dev_scale_factor * distance)
        return (std_dev, std_dev, std_dev * 2)
