# Internal imports
from config import OperatorRobotConfig
from subsystem.drivetrain.swerve_drivetrain import SwerveDrivetrain
from subsystem.diverCarlElevator import DiverCarlElevator
from subsystem.diverCarlChistera import DiverCarlChistera

# Third-party imports
import wpilib
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpiutil.log import (
    BooleanLogEntry, StringLogEntry, FloatLogEntry, IntegerLogEntry
)

telemetryButtonEntries = [
    ["AButton", BooleanLogEntry, "/button/letter/a"],
    ["BButton", BooleanLogEntry, "/button/letter/b"],
    ["XButton", BooleanLogEntry, "/button/letter/x"],
    ["YButton", BooleanLogEntry, "/button/letter/y"],
    ["BackButton", BooleanLogEntry, "/button/front/back"],
    ["StartButton", BooleanLogEntry, "/button/front/start"],
    ["LeftBumper", BooleanLogEntry, "/button/bumper/left"],
    ["RightBumper", BooleanLogEntry, "/button/bumper/right"],
    ["LeftStickButton", BooleanLogEntry, "button/stickbutton/left"],
    ["RightStickButton", BooleanLogEntry, "button/stickbutton/right"],
    ["RightTrigger", FloatLogEntry, "button/trigger/right"],
    ["LeftTrigger", FloatLogEntry, "button/trigger/left"],
    ["JoystickLeftY", FloatLogEntry, "button/joystick/lefty"],
    ["JoystickLeftX", FloatLogEntry, "button/joystick/leftx"],
    ["JoystickRightY", FloatLogEntry, "button/joystick/righty"],
    ["JoystickRightX", FloatLogEntry, "button/joystick/rightx"],
    ["DPad", IntegerLogEntry, "button/dpad"]
]

telemetryOdometryEntries = [
    ["robotPose", "robotpose"],
    ["targetPose", "targetpose"],
]

telemetryFullSwerveDriveTrainEntries = [
    ["moduleStates", SwerveModuleState, True, "swervemodeulestates"],
    ["drivetrainVelocity", ChassisSpeeds, False, "swervevelocity"],
    ["drivetrainRotation", Rotation2d, False, "swerverotation"]
]

telemetryRawSwerveDriveTrainEntries = []
for i in range(len(OperatorRobotConfig.swerve_module_channels)):
    telemetryRawSwerveDriveTrainEntries.extend([
        [f"steerDegree{i + 1}", FloatLogEntry, f"module{i + 1}/steerdegree"],
        [f"drivePercent{i + 1}", FloatLogEntry, f"module{i + 1}/drivepercent"],
        [f"moduleVelocity{i + 1}", FloatLogEntry, f"module{i + 1}/velocity"],
    ])

elevatorEntries = [
        # self.current_height_above_zero = self.encoder.getPosition()
        # self.current_height = self.height_at_zero + self.current_height_above_zero
        # self.current_goal_height = self.height_at_zero
        # self.current_goal_height_above_zero = 0
        # self.at_goal = self.checkIfAtGoalHeight()
        # self.error_from_goal = self.current_height_above_zero - self.current_goal_height_above_zero
        # self.at_top_limit = self.motor.getForwardLimitSwitch().get()
        # self.at_bottom_limit = self.motor.getReverseLimitSwitch().get()
        # self.motor_current = self.motor.getOutputCurrent()
        # self.motor_output = self.motor.getAppliedOutput()
        # self.motor_velocity = self.encoder.getVelocity()
    ["elevatorCurrentHeightAboveZero", FloatLogEntry, "/positions"],
    ["elevatorCurrentHeight", FloatLogEntry, "/positions"],
    ["elevatorCurrentGoalHeight", FloatLogEntry, "/positions"],
    ["elevatorCurrentGoalHeightAboveZero", FloatLogEntry, "/positions"],
    ["elevatorAtGoal", BooleanLogEntry, "/positions/atgoal"],
    ["elevatorErrorFromGoal", FloatLogEntry, "/positions"],
    ["elevatorAtTopLimit", BooleanLogEntry, "/limits"],
    ["elevatorAtBottomLimit", BooleanLogEntry, "/limits"],
    ["elevatorMotorCurrent", FloatLogEntry, "/output"],
    ["elevatorMotorOutput", FloatLogEntry, "/output"],
    ["elevatorMotorVelocity", FloatLogEntry, "/output"],
]

armEntries = [
    ["armPositionArc", FloatLogEntry, "/positions"],
    ["armSetArc", FloatLogEntry, "/positions"],
    ["armReqArc", FloatLogEntry, "/positions"],
    ["armAtGoal", BooleanLogEntry, "/positions"],
    ["armError", FloatLogEntry, "/positions"],
    ["armAtHardlimit", BooleanLogEntry, "/limits"],
    ["armDisabled", BooleanLogEntry, "/output"]
]

driverStationEntries = [
    ["alliance", StringLogEntry, "alliance"],
    ["autonomous", BooleanLogEntry, "autonomous"],
    ["teleop", BooleanLogEntry, "teleop"],
    ["test", BooleanLogEntry, "test"],
    ["enabled", BooleanLogEntry, "enabled"]
]

class Telemetry:

    def __init__(
        self,
        driverController: wpilib.XboxController = None,
        mechController: wpilib.XboxController = None,
        driveTrain: SwerveDrivetrain = None,
        elevator: DiverCarlElevator = None,
        driverStation: wpilib.DriverStation = None,
        arm: DiverCarlChistera = None,
    ):
        self.driverController = driverController
        self.mechController = mechController
        self.odometryPosition = driveTrain.pose_estimator
        self.driveTrain = driveTrain
        self.swerveModules = driveTrain.swerve_modules
        self.elevator = elevator
        self.driverStation = driverStation
        self.arm = arm

        self.networkTable = NetworkTableInstance.getDefault()
        for entryname, logname in telemetryOdometryEntries:
            setattr(self, entryname, self.networkTable.getStructTopic("odometry/" + logname, Pose2d).publish())
        for entryname, entrytype, isarraytype, logname in telemetryFullSwerveDriveTrainEntries:
            if isarraytype:
                setattr(self, entryname, self.networkTable.getStructArrayTopic("swervedrivetrain/" + logname, entrytype).publish())
            else:
                setattr(self, entryname, self.networkTable.getStructTopic("swervedrivetrain/" + logname, entrytype).publish())

        self.datalog = wpilib.DataLogManager.getLog()
        for entryname, entrytype, logname in telemetryButtonEntries:
            setattr(self, "driver" + entryname, entrytype(self.datalog, "driver/" + logname))
            setattr(self, "mech" + entryname, entrytype(self.datalog, "mech/" + logname))
        for entryname, entrytype, logname in telemetryRawSwerveDriveTrainEntries:
            setattr(self, entryname, entrytype(self.datalog, "rawswervedrivetrain/" + logname))
        for entryname, entrytype, logname in elevatorEntries:
            setattr(self, entryname, entrytype(self.datalog, "elevator/" + logname))
        for entryname, entrytype, logname in driverStationEntries:
            setattr(self, entryname, entrytype(self.datalog, "driverstation/" + logname))
        for entryname, entrytype, logname in armEntries:
            setattr(self, entryname, entrytype(self.datalog, "arm/" + logname))

        if self.driverStation:
            self.driverStation.startDataLog(self.datalog)

    def getDriverControllerInputs(self):
        """
        Records data for buttons and axis inputs for the first controller
        """
        self.driverAButton.append(self.driverController.getAButton()) #bool
        self.driverBButton.append(self.driverController.getBButton()) #bool
        self.driverXButton.append(self.driverController.getXButton()) #bool
        self.driverYButton.append(self.driverController.getYButton()) #bool
        self.driverBackButton.append(self.driverController.getBackButton()) #left side , bool
        self.driverStartButton.append(self.driverController.getStartButton()) #right side , bool
        self.driverLeftBumper.append(self.driverController.getLeftBumper()) #bool
        self.driverRightBumper.append(self.driverController.getRightBumper()) #bool
        self.driverLeftStickButton.append(self.driverController.getLeftStickButton()) #bool
        self.driverRightStickButton.append(self.driverController.getRightStickButton()) #bool
        self.driverLeftTrigger.append(self.driverController.getLeftTriggerAxis()) #float 0-1
        self.driverRightTrigger.append(self.driverController.getRightTriggerAxis()) #float 0-1
        self.driverJoystickLeftY.append(self.driverController.getLeftY()) #float -1-1
        self.driverJoystickLeftX.append(self.driverController.getLeftX()) #float -1-1
        self.driverJoystickRightY.append(self.driverController.getRightY()) #float -1-1
        self.driverJoystickRightX.append(self.driverController.getRightX()) #float -1-1
        self.driverDPad.append(self.driverController.getPOV()) #ints

    def getMechControllerInputs(self):
        """
        Records data for buttons and axis inputs for the second controller
        """
        self.mechAButton.append(self.mechController.getAButton()) #bool
        self.mechBButton.append(self.mechController.getBButton()) #bool
        self.mechXButton.append(self.mechController.getXButton()) #bool
        self.mechYButton.append(self.mechController.getYButton()) #bool
        self.mechBackButton.append(self.mechController.getBackButton()) #left side , bool
        self.mechStartButton.append(self.mechController.getStartButton()) #right side , bool
        self.mechLeftBumper.append(self.mechController.getLeftBumper()) #bool
        self.mechRightBumper.append(self.mechController.getRightBumper()) #bool
        self.mechLeftStickButton.append(self.mechController.getLeftStickButton()) #bool
        self.mechRightStickButton.append(self.mechController.getRightStickButton()) #bool
        self.mechLeftTrigger.append(self.mechController.getLeftTriggerAxis()) #float 0-1
        self.mechRightTrigger.append(self.mechController.getRightTriggerAxis()) #float 0-1
        self.mechJoystickLeftY.append(self.mechController.getLeftY()) #float -1-1
        self.mechJoystickLeftX.append(self.mechController.getLeftX()) #float -1-1
        self.mechJoystickRightY.append(self.mechController.getRightY()) #float -1-1
        self.mechJoystickRightX.append(self.mechController.getRightX()) #float -1-1
        self.mechDPad.append(self.mechController.getPOV()) #ints

    def getOdometryInputs(self):
        """
        Records the data for the positions of the bot in a field,
        Gives the x position, y position and rotation
        """
        pose = self.odometryPosition.getEstimatedPosition()
        self.robotPose.set(pose)

    def getFullSwerveState(self):
        """
        Retrieves values reflecting the current state of the swerve drive
        """
        self.moduleStates.set([swerveModule.current_state() for swerveModule in self.swerveModules])
        self.drivetrainVelocity.set(self.driveTrain.current_robot_relative_speed())
        self.drivetrainRotation.set(self.driveTrain.current_yaw())

    def getRawSwerveInputs(self):
        """
        Gets the inputs for some swerve drive train inputs
        it get the steer angle, the drive percent and the velocity
        """
        for i, swerveModule in enumerate(self.swerveModules):
            getattr(self, f"steerDegree{i + 1}").append(swerveModule.current_raw_absolute_steer_position())
            getattr(self, f"drivePercent{i + 1}").append(swerveModule.drive_motor.getAppliedOutput())
            getattr(self, f"moduleVelocity{i + 1}").append(swerveModule.current_state().speed)

    def getElevatorInputs(self):
        """
        Retrieves values reflecting the current state of the elevator and information about
        the goal position of the elevator
        """
        self.elevatorCurrentHeightAboveZero.append(self.elevator.current_height_above_zero)
        self.elevatorCurrentHeight.append(self.elevator.current_height)
        self.elevatorCurrentGoalHeight.append(self.elevator.current_goal_height)
        self.elevatorCurrentGoalHeightAboveZero.append(self.elevator.current_goal_height_above_zero)
        self.elevatorAtGoal.append(self.elevator.at_goal)
        self.elevatorErrorFromGoal.append(self.elevator.error_from_goal)
        self.elevatorAtTopLimit.append(self.elevator.at_top_limit)
        self.elevatorAtBottomLimit.append(self.elevator.at_bottom_limit)
        self.elevatorMotorCurrent.append(self.elevator.motor_current)
        self.elevatorMotorOutput.append(self.elevator.motor_output)
        self.elevatorMotorVelocity.append(self.elevator.motor_velocity)

    def getDriverStationInputs(self):
        """
        Gets the inputs of some match/general robot things,
        the things being: Alliance color and what mode it is in and
        if it is enabled
        """
        alliance = "No Alliance"
        if self.driverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue:
            alliance = "Blue"
        if self.driverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            alliance = "Red"
        self.alliance.append(alliance)
        self.autonomous.append(self.driverStation.isAutonomous())
        self.teleop.append(self.driverStation.isTeleop())
        self.test.append(self.driverStation.isTest())
        self.enabled.append(self.driverStation.isEnabled())

    def getArmInputs(self):
        self.armPositionArc.append(self.arm.getArc())
        self.armSetArc.append(self.arm.getSetArc())
        self.armReqArc.append(self.arm._requestedGoal)
        self.armAtGoal.append(self.arm.atGoal())
        self.armError.append(self.arm.getError())
        self.armAtHardlimit.append(self.arm.getForwardLimit())
        self.armDisabled.append(self.arm.getDisabled())

    def runDefaultDataCollections(self):
        if self.driverController is not None:
            self.getDriverControllerInputs()
        if self.mechController is not None:
            self.getMechControllerInputs()
        if self.odometryPosition is not None:
            self.getOdometryInputs()
        if self.driveTrain and self.swerveModules:
            self.getFullSwerveState()
        if self.swerveModules is not None:
            self.getRawSwerveInputs()
        if self.elevator is not None:
            self.getElevatorInputs()
        if self.driverStation is not None:
            self.getDriverStationInputs()

    def logAdditionalOdometry(self, odometer_value: Pose2d, log_entry_name: str) -> None:
        getattr(self, log_entry_name).set(odometer_value)
