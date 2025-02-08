from typing import Optional

from wpilib import SmartDashboard
from wpimath.geometry import Transform3d, Rotation3d, Translation3d
from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy
from photonlibpy.targeting.photonTrackedTarget import PhotonTrackedTarget
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

from subsystem.drivetrain.swerve_drivetrain import SwerveDrivetrain
from config import OperatorRobotConfig

class Vision:
    def __init__(self, driveTrain: SwerveDrivetrain, cam_name : str = "Sparky_Arducam_1"):
        self.cam = PhotonCamera(cam_name)
        self.drive = driveTrain
        self.camPoseEst = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape),
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
            self.drive.add_vision_pose_estimate(
                camEstPose.estimatedPose.toPose2d(), camEstPose.timestampSeconds
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