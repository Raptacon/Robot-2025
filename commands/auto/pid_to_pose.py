# Native imports
from math import inf
from typing import Callable

# Internal imports
from config import OperatorRobotConfig
from subsystem.drivetrain.swerve_drivetrain import SwerveDrivetrain

# Third-party imports
from commands2 import Command
from wpimath import applyDeadband
from wpimath.controller import ProfiledPIDController
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.trajectory import TrapezoidProfile


class PIDToPose(Command):
    def __init__(
        self,
        drivetrain: SwerveDrivetrain,
        target_pose: Callable[[], Pose2d] | None,
        translation_pid_config: tuple = OperatorRobotConfig.pid_to_pose_translation_pid_profile,
        rotation_pid_config: tuple = OperatorRobotConfig.pid_to_pose_rotation_pid_profile,
        setpoint_tolerances: tuple = OperatorRobotConfig.pid_to_pose_setpoint_tolerances
    ) -> None:
        """
        """
        super().__init__()

        self.drivetrain = drivetrain
        self.target_pose = target_pose
        self.x_translation_pid = ProfiledPIDController(
            *translation_pid_config[0:3], TrapezoidProfile.Constraints(*translation_pid_config[3:5])
        )
        self.y_translation_pid = ProfiledPIDController(
            *translation_pid_config[0:3], TrapezoidProfile.Constraints(*translation_pid_config[3:5])
        )
        self.rotation_pid = ProfiledPIDController(
            *rotation_pid_config[0:3], TrapezoidProfile.Constraints(*rotation_pid_config[3:5])
        )

        self.rotation_pid.enableContinuousInput(-180, 180)

        self.x_translation_pid.setTolerance(setpoint_tolerances[0])
        self.y_translation_pid.setTolerance(setpoint_tolerances[1])
        self.rotation_pid.setTolerance(setpoint_tolerances[2])

        self.addRequirements(self.drivetrain)

    def execute(self) -> None:
        """
        """
        if self.target_pose:
            current_pose = self.drivetrain.current_pose()
            
            x_pose_error = self.target_pose().X() - current_pose.X()
            y_pose_error = self.target_pose().Y() - current_pose.Y()
            rotation_error = (self.target_pose().rotation() - current_pose.rotation()).degrees()

            x_output = -applyDeadband(self.x_translation_pid.calculate(x_pose_error, 0), 0.04, inf)
            y_output = -applyDeadband(self.y_translation_pid.calculate(y_pose_error, 0), 0.04, inf)
            rotation_output = -applyDeadband(self.rotation_pid.calculate(rotation_error, 0), 0.04, inf)

            drive_speed = ChassisSpeeds.fromFieldRelativeSpeeds(x_output, y_output, rotation_output, current_pose.rotation())
            self.drivetrain.drive(drive_speed.vx, drive_speed.vy, drive_speed.omega, False)

    def end(self, interrupted: bool) -> None:
        """
        """
        self.drivetrain.stop_driving(apply_to_modules=True)

    def isFinished(self) -> None:
        """
        """
        at_setpoints = self.x_translation_pid.atSetpoint() and self.y_translation_pid.atSetpoint() and self.rotation_pid.atSetpoint()
        no_target_pose = True if not self.target_pose else False
        return at_setpoints or no_target_pose
