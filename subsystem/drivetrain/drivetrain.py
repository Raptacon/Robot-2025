# Native imports
from typing import Tuple

# Internal imports
from config import OperatorRobotConfig
from constants import SwerveDriveConsts
from .swerve_module import SwerveModuleMk4iSparkMaxFalconCanCoder

# Third-party imports
from wpilib import DriverStation, SmartDashboard
from wpimath.system.plant import DCMotor
import navx
from commands2 import Subsystem
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import ModuleConfig, RobotConfig, PIDConstants
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Rotation3d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveModulePosition


class SwerveDrivetrain(Subsystem):
    """
    """
    def __init__(
        self,
        starting_pose: Pose2d = Pose2d(
            Translation2d(*OperatorRobotConfig.default_start_pose[0:2]),
            Rotation2d.fromDegrees(OperatorRobotConfig.default_start_pose[2])
        )) -> None:
        """
        """
        self.constants = SwerveDriveConsts()
        self.invert_gyro = False

        # must do in front-left, front-right, back-left, back-right order
        self.swerve_modules = [
            SwerveModuleMk4iSparkMaxFalconCanCoder(
                "frontLeft", (self.constants.moduleFrontLeftX, self.constants.moduleFrontLeftY),
                OperatorRobotConfig.swerve_module_channels[0], invert_steer=True, invert_drive=True, encoder_calibration=OperatorRobotConfig.swerve_abs_encoder_calibrations[0]
            ),
            SwerveModuleMk4iSparkMaxFalconCanCoder(
                "frontRight", (self.constants.moduleFrontRightX, self.constants.moduleFrontRightY),
                OperatorRobotConfig.swerve_module_channels[1], invert_steer=True, invert_drive=True, encoder_calibration=OperatorRobotConfig.swerve_abs_encoder_calibrations[1]
            ),
            SwerveModuleMk4iSparkMaxFalconCanCoder(
                "backLeft", (self.constants.moduleBackLeftX, self.constants.moduleBackLeftY),
                OperatorRobotConfig.swerve_module_channels[2], invert_steer=True, invert_drive=True, encoder_calibration=OperatorRobotConfig.swerve_abs_encoder_calibrations[2]
            ),
            SwerveModuleMk4iSparkMaxFalconCanCoder(
                "backRight", (self.constants.moduleBackRightX, self.constants.moduleBackRightY),
                OperatorRobotConfig.swerve_module_channels[3], invert_steer=True, invert_drive=True, encoder_calibration=OperatorRobotConfig.swerve_abs_encoder_calibrations[3]
            )
        ]

        self.drive_kinematics = SwerveDrive4Kinematics(
            *[swerve_module.drivetrain_location for swerve_module in self.swerve_modules] # -> Translation2d()
        )

        self.gyroscope = navx.AHRS.create_spi()
        self.heading_offset = Rotation3d()
        self.factory_default_gyro()

        self.pose_estimator = SwerveDrive4PoseEstimator(
            self.drive_kinematics,
            self.current_yaw(),
            self.current_module_positions(),
            starting_pose
        )

        self.starting_pose = starting_pose
        self.reset_heading()

        # Path Planner setup
        #path_planner_config = self.gen_path_planner_config()
        #path_planner_config = RobotConfig.fromGUISettings()
        #self.configure_path_planner(path_planner_config)

    def raw_current_heading(self) -> Rotation3d:
        """
        """
        return -self.gyroscope.getRotation3d() if self.invert_gyro else self.gyroscope.getRotation3d()

    def current_heading(self) -> Rotation3d:
        """
        """
        return self.raw_current_heading() - self.heading_offset

    def current_yaw(self) -> Rotation2d:
        """
        """
        return Rotation2d(self.current_heading().Z())

    def factory_default_gyro(self) -> None:
        """
        """
        self.heading_offset = self.gyroscope.getRotation3d()

    def reset_heading(self) -> None:
        """
        """
        self.heading_offset = self.raw_current_heading()
        self.reset_pose_estimator(Pose2d(self.current_pose().translation(), Rotation2d()))

    def drive(
        self,
        velocity_vector_x: float,
        velocity_vector_y: float,
        angular_velocity: float,
        field_relative: bool = False
    ) -> None:
        """
        """
        if field_relative:
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                velocity_vector_x, velocity_vector_y, angular_velocity, self.current_yaw()
            )
        else:
            chassis_speeds = ChassisSpeeds(velocity_vector_x, velocity_vector_y, angular_velocity)

        self.set_states_from_speeds(chassis_speeds)

    def current_module_positions(self) -> Tuple[SwerveModulePosition]:
        """
        """
        return tuple([swerve_module.current_position() for swerve_module in self.swerve_modules])

    def current_pose(self) -> Pose2d:
        """
        """
        return self.pose_estimator.getEstimatedPosition()

    def current_robot_relative_speed(self) -> ChassisSpeeds:
        """
        """
        return self.drive_kinematics.toChassisSpeeds(tuple(
            swerve_module.current_state() for swerve_module in self.swerve_modules
        ))

    def set_states_from_speeds(self, drivetrain_speeds: ChassisSpeeds) -> None:
        """
        """
        module_states = self.drive_kinematics.toSwerveModuleStates(drivetrain_speeds)
        module_states = self.drive_kinematics.desaturateWheelSpeeds(module_states, self.constants.maxTranslationMPS)

        for i, module_state in enumerate(module_states):
            self.swerve_modules[i].set_state(module_state)

    def update_pose_estimator(self) -> None:
        """
        """
        self.pose_estimator.update(self.current_yaw(), self.current_module_positions())
        for swerve_module in self.swerve_modules:
            swerve_module.update_telemetry()
        SmartDashboard.putNumber("Drivetrain Raw IMU Yaw", self.current_yaw().degrees())
        SmartDashboard.putNumber("Drivetrain Unadjusted IMU Yaw", self.gyroscope.getRotation3d().Z())
        SmartDashboard.putNumber("Drivetrain Heading Offset", self.heading_offset.Z())
        SmartDashboard.putNumber("Starting angle", self.starting_pose.rotation().degrees() % 360.0)
        SmartDashboard.putNumber("Odometry: X Pose", self.current_pose().X())
        SmartDashboard.putNumber("Odometry: Y Pose", self.current_pose().Y())
        SmartDashboard.putNumber("Odometry: Angle Pose", self.current_pose().rotation().degrees())

    def reset_pose_estimator(self, current_pose: Pose2d) -> None:
        """
        """
        self.pose_estimator.resetPosition(self.current_yaw(), self.current_module_positions(), current_pose)
        self.stop_driving(apply_to_modules=False)

    def stop_driving(self, apply_to_modules: bool = True) -> None:
        """
        """
        robot_relative_speeds = ChassisSpeeds.fromRobotRelativeSpeeds(ChassisSpeeds(0, 0, 0), self.current_yaw())
        module_states = self.drive_kinematics.toSwerveModuleStates(robot_relative_speeds)

        if apply_to_modules:
            for i, module_state in enumerate(module_states):
                self.swerve_modules[i].set_state(module_state)

    def set_motor_stop_modes(self, to_drive: bool, to_break: bool, all_motor_override: bool = False, burn_flash: bool = False) -> bool:
        """
        """
        for swerve_module in self.swerve_modules:
            swerve_module.set_motor_stop_mode(to_drive=to_drive, to_break=to_break)
            swerve_module.apply_motor_config(to_drive=to_drive, burn_flash=burn_flash)
            if all_motor_override:
                swerve_module.set_motor_stop_mode(to_drive=not to_drive, to_break=to_break)
                swerve_module.apply_motor_config(to_drive=not to_drive, burn_flash=burn_flash)

    def flip_to_red_alliance(self) -> bool:
        """
        """
        alliance = DriverStation.getAlliance()
        if alliance:
            return alliance == DriverStation.Alliance.kRed
        return False

    def gen_path_planner_config(self) -> RobotConfig:
        """
        """
        path_planner_config = RobotConfig(
            massKG=self.constants.massKG,
            MOI=self.constants.MOI,
            moduleConfig=ModuleConfig(
                wheelRadiusMeters=self.swerve_modules[0].constants.wheelDiameter / 2.0,
                maxDriveVelocityMPS=self.constants.maxTranslationMPS,
                wheelCOF=self.swerve_modules[0].constants.wheelCOF,
                driveMotor=getattr(DCMotor, self.swerve_modules[0].constants.motorType)(
                    self.swerve_modules[0].constants.numDriveMotors
                ),
                driveCurrentLimit=self.swerve_modules[0].constants.kDriveCurrentLimit,
                numMotors=self.swerve_modules[0].constants.numDriveMotors,
            ),
            moduleOffsets=[swerve_module.drivetrain_location for swerve_module in self.swerve_modules],
        )

        return path_planner_config

    def configure_path_planner(self, config: RobotConfig) -> None:
        """
        """
        AutoBuilder.configure(
            self.current_pose,
            self.reset_pose_estimator,
            self.current_robot_relative_speed,
            lambda speeds, feedforwards: self.set_states_from_speeds(speeds),
            PPHolonomicDriveController(
                PIDConstants(*OperatorRobotConfig.pathplanner_translation_pid),
                PIDConstants(*OperatorRobotConfig.pathplanner_rotation_pid)
            ),
            config,
            self.flip_to_red_alliance,
            self
        )

    def periodic(self) -> None:
        """
        """
        self.update_pose_estimator()
