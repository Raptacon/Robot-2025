from typing import Callable, Optional, Tuple

from wpilib import SmartDashboard
from wpimath.geometry import Pose2d, Transform3d, Rotation3d, Translation3d
from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy
from photonlibpy.targeting import PhotonPipelineResult, PhotonTrackedTarget
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
        self.cameras = [PhotonCamera(left_cam_name), PhotonCamera(right_cam_name)]
        #self.cam_left = PhotonCamera(left_cam_name)
        #self.cam_right = PhotonCamera(right_cam_name)
        self.drive = driveTrain
        self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)
        self.reef_tag_ids = {positions["tag"] for positions in reef_position_lookup.values()}

        # self.camPoseEstLeft = self.camPoseEstRight = 
        self.cameraPoseEstimators = [
            PhotonPoseEstimator(
                self.field_layout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                Transform3d(Translation3d(*camToRobotTranslation), Rotation3d.fromDegrees(*camToRobotRotation))
            )
            for camera, camToRobotTranslation, camToRobotRotation in zip(
                self.cameras, [
                    (OperatorRobotConfig.robot_Cam_Translation_Left, OperatorRobotConfig.robot_Cam_Rotation_Degress_Left),
                    (OperatorRobotConfig.robot_Cam_Translation_Right, OperatorRobotConfig.robot_Cam_Rotation_Degress_Right)
                ]
            )
        ]

        self.cameraPoseEstimates = [None] * len(self.cameras)

    def getSingleCamEstimate(
        self, camera: PhotonCamera, poseEstimator: PhotonPoseEstimator, specificTagId: int | None = None
    ) -> Pose2d | None:
        poseEstimate = None
        validTagIds = self.reef_tag_ids
        if specificTagId != None:
            validTagIds = {specificTagId}

        if not ((camera is None) or (poseEstimator is None)):
            unreadPipelines = camera.getAllUnreadResults()
            if len(unreadPipelines) > 0:
                bestPipeline = unreadPipelines[-1]
                targetsKeep = [
                    target
                    for target in bestPipeline.getTargets()
                    if (
                        (target is not None)
                        and (target.getFiducialId() in validTagIds)
                        and (target.getPoseAmbiguity() < OperatorRobotConfig.vision_ambiguity_threshold)
                        and (target.getBestCameraToTarget().translation().norm() < OperatorRobotConfig.vision_distance_threshold_m)
                    )
                ]

                if len(targetsKeep) > 0:
                    filteredPipeline = PhotonPipelineResult(
                        bestPipeline.ntReceiveTimestampMicros, targetsKeep, bestPipeline.metadata
                    )
                    camEstPose = poseEstimator.update(filteredPipeline)

                    if camEstPose:
                        targetDistances = [
                            target.getBestCameraToTarget().translation().norm() for target in filteredPipeline.getTargets()
                        ]

                        if len(targetDistances) > 0:
                            distanceToClosestTarget = min(targetDistances)
                            stdDev = self.distanceToStdDev(distanceToClosestTarget)
                            poseEstimate = camEstPose.estimatedPose.toPose2d()
                            self.drive.add_vision_pose_estimate(
                                poseEstimate, camEstPose.timestampSeconds, stdDev
                            )
        return poseEstimate

    def getCamEstimates(self, specificTagId: Optional[Callable[[], int]] = None) -> None:
        for i, camera, cameraPoseEstimator in enumerate(zip(self.cameras, self.cameraPoseEstimators)):
            poseEstimate = self.getSingleCamEstimate(camera, cameraPoseEstimator, specificTagId=specificTagId())
            self.cameraPoseEstimates[i] = poseEstimate

    def getTargetData(self, target: PhotonTrackedTarget) -> tuple[float, float, float, float]:
        if target is None:
            targetID, targetYaw, targetPitch, targetAmbiguity == (0, 0, 0, 0)
        else:
            targetID = target.getFiducialId()
            targetYaw = target.getYaw()
            targetPitch = target.getPitch()
            targetAmbiguity = target.getPoseAmbiguity()
        return targetID, targetYaw, targetPitch, targetAmbiguity

    def showTargetData(self, target: Optional[PhotonTrackedTarget] = None):
        if target is None:
            pipelineResults = self.cameras[0].getAllUnreadResults()
            if len(pipelineResults) > 0:
                target = pipelineResults[-1].getBestTarget()

                if target is None:
                    return

                targetID, targetYaw, targetPitch, targetAmbiguity = self.getTargetData(target)

                SmartDashboard.putNumber("Target ID", targetID)
                SmartDashboard.putNumber("Target Yaw", targetYaw)
                SmartDashboard.putNumber("Target Pitch", targetPitch)
                SmartDashboard.putNumber("Target Ambiguity", targetAmbiguity)

    def distanceToStdDev(self, distance: float | None) -> Tuple[float]:
        std_dev = OperatorRobotConfig.vision_default_std_dev
        if distance:
            if distance > OperatorRobotConfig.vision_distance_threshold_m:
                # Ignore vision if too far away from tag
                std_dev = 10000
            else:
                std_dev = -1 + (OperatorRobotConfig.vision_std_dev_basis)**(OperatorRobotConfig.vision_std_dev_scale_factor * distance)
        return (std_dev, std_dev, std_dev * 2)
