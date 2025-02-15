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
    def __init__(self, driveTrain: SwerveDrivetrain, cam_name : str = "Sparky_Arducam_1"):
        self.cam = PhotonCamera(cam_name)
        self.drive = driveTrain
        self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)
        self.camPoseEst = PhotonPoseEstimator(
            self.field_layout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.cam,
            Transform3d(
                Translation3d(
                    OperatorRobotConfig.robot_Cam_Translation[0],
                    OperatorRobotConfig.robot_Cam_Translation[1],
                    OperatorRobotConfig.robot_Cam_Translation[2]
                ),
                Rotation3d.fromDegrees(
                    OperatorRobotConfig.robot_Cam_Rotation_Degress[0],
                    OperatorRobotConfig.robot_Cam_Rotation_Degress[1],
                    OperatorRobotConfig.robot_Cam_Rotation_Degress[2]
                )
            )
        )

    def getCamEstimate(self):
        bestPipeline = self.cam.getLatestResult()
        camEstPose = self.camPoseEst.update(bestPipeline)
        if camEstPose:
            robot_pose = camEstPose.estimatedPose.toPose2d()

            tag_distances = [
                PhotonUtils.getDistanceToPose(robot_pose, self.field_layout.getTagPose(targ.getFiducialID()).toPose2d())
                for targ in bestPipeline.getTargets()
                if targ is not None
            ]

            distance_to_closest_tag = None
            if len(distance_to_closest_tag) > 0:
                distance_to_closest_tag = min(tag_distances)

            std_dev = self.distanceToStdDev(distance_to_closest_tag)

            self.drive.add_vision_pose_estimate(
                camEstPose.estimatedPose.toPose2d(), camEstPose.timestampSeconds, std_dev
            )

    def getTargetData(self, target : PhotonTrackedTarget) -> tuple[float, float, float, float]:
        targetID = target.getFiducialId()
        targetYaw = target.getYaw()
        targetPitch = target.getPitch()
        targetAmbiguity = target.getPoseAmbiguity()
        return targetID, targetYaw, targetPitch, targetAmbiguity
    
    def showTargetData(self, target : Optional[PhotonTrackedTarget] = None):
        if target == None:
            target = self.cam.getLatestResult().getBestTarget()

        if(not self.cam.getLatestResult().hasTargets() or self.cam.getLatestResult() == None):
            return

        targetID, targetYaw, targetPitch, targetAmbiguity = self.getTargetData(target)

        SmartDashboard.putNumber("Target ID", targetID)
        SmartDashboard.putNumber("Target Yaw", targetYaw)
        SmartDashboard.putNumber("Target Pitch", targetPitch)
        SmartDashboard.putNumber("Target Ambiguity", targetAmbiguity)

    def distanceToStdDev(self, distance: float | None) -> Tuple[float]:
        std_dev = 0.9
        if distance:
            std_dev = -1 + (OperatorRobotConfig.vision_std_dev_basis)**(OperatorRobotConfig.vision_std_dev_scale_factor * distance)
        return (std_dev, std_dev, std_dev)