# Native imports
import json
import os
from pathlib import Path
from typing import Callable

# Internal imports
from data.telemetry import Telemetry
from constants import DiverCarlElevatorConsts, PoseOptions
#from vision import Vision
from commands.auto.pathplan_to_pose import pathplanToPose
from commands.default_swerve_drive import DefaultDrive
import commands.operate_elevator as elevCommands
from commands.operate_elevator import ElevateManually, ElevateToGoal
from lookups.utils import getCurrentReefZone
from lookups.reef_positions import reef_position_lookup, reef_height_lookup
from subsystem.drivetrain.swerve_drivetrain import SwerveDrivetrain
from subsystem.captainIntake import CaptainIntake, CaptainIntakeStateMachine

# Third-party imports
import commands2
import ntcore
import wpilib
import wpimath
from commands2.button import Trigger
from pathplannerlib.auto import AutoBuilder, NamedCommands
from pathplannerlib.path import PathPlannerPath
from subsystem.diverCarlElevator import DiverCarlElevator as Elevator
from subsystem.diverCarlChistera import DiverCarlChistera as Arm

class RobotSwerve:
    """
    Container to hold the main robot code
    """
    # forward declare critical types for editors
    drivetrain: SwerveDrivetrain
    elevator: Elevator
    arm: Arm

    def __init__(self, is_disabled: Callable[[], bool]) -> None:
        # networktables setup
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.table = self.inst.getTable("Stream_Deck")

        # Subsystem instantiation
        self.drivetrain = SwerveDrivetrain()
        self.elevator = Elevator()
        self.arm = Arm()
        # cross link arm and elevator
        self.arm.setElevator(self.elevator)
        self.elevator.setArm(self.arm)
        self.alliance = "red" if self.drivetrain.flip_to_red_alliance() else "blue"

        self.intake_subsystem = CaptainIntake()
        self.intake_state_machine = CaptainIntakeStateMachine(self.intake_subsystem)
        self.intake_command_scheduler = commands2.CommandScheduler.getInstance()

        # Initialize timer
        self.timer = wpilib.Timer()
        self.timer.start()

        # HID setup
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)
        self.driver_controller = wpilib.XboxController(0)
        self.mech_controller = wpilib.XboxController(1)

        # Register Named Commands
        NamedCommands.registerCommand('Raise_Place', elevCommands.genPivotElevatorCommand(self.arm, self.elevator, PoseOptions.REEF4))
        NamedCommands.registerCommand('Raise_Place_L1', elevCommands.genPivotElevatorCommand(self.arm, self.elevator, PoseOptions.TROUGH))
        NamedCommands.registerCommand('Raise_Place_L2', elevCommands.genPivotElevatorCommand(self.arm, self.elevator, PoseOptions.REEF2))
        NamedCommands.registerCommand('Raise_Place_L3', elevCommands.genPivotElevatorCommand(self.arm, self.elevator, PoseOptions.REEF3))
        NamedCommands.registerCommand('Raise_Place_L4', elevCommands.genPivotElevatorCommand(self.arm, self.elevator, PoseOptions.REEF4))
        NamedCommands.registerCommand('Coral_Intake', ElevateToGoal(self.elevator, DiverCarlElevatorConsts.kChuteHeightCm))

        # Autonomous setup
        self.auto_command = None
        self.auto_chooser = AutoBuilder.buildAutoChooser()
        wpilib.SmartDashboard.putData("Select auto routine", self.auto_chooser)

        self.teleop_stem_paths = {
            start_location: PathPlannerPath.fromPathFile(start_location)
            for start_location in [f"Stem_Reef_F{n}" for n in range(1, 7)] + [f"Stem_Reef_N{n}" for n in range(1, 7)]
        }

        # Telemetry setup
        self.enableTelemetry = wpilib.SmartDashboard.getBoolean("enableTelemetry", True)
        if self.enableTelemetry:
            self.telemetry = Telemetry(self.driver_controller, self.mech_controller, self.drivetrain, self.elevator, wpilib.DriverStation)

        wpilib.SmartDashboard.putString("Robot Version", self.getDeployInfo("git-hash"))
        wpilib.SmartDashboard.putString("Git Branch", self.getDeployInfo("git-branch"))
        wpilib.SmartDashboard.putString(
            "Deploy Host", self.getDeployInfo("deploy-host")
        )
        wpilib.SmartDashboard.putString(
            "Deploy User", self.getDeployInfo("deploy-user")
        )

        # Vision setup
        #self.vision = Vision(self.drivetrain)

        # Update drivetrain motor idle modes 3 seconds after the robot has been disabled.
        # to_break should be False at competitions where the robot is turned off between matches
        Trigger(is_disabled()).debounce(3).onTrue(
            commands2.cmd.runOnce(
                self.drivetrain.set_motor_stop_modes(
                    to_drive=True, to_break=True, all_motor_override=True, burn_flash=True
                ),
                self.drivetrain
            )
        )

    def robotPeriodic(self):
        if self.enableTelemetry and self.telemetry:
            self.telemetry.runDefaultDataCollections()

        self.intake_command_scheduler.run()

        #self.vision.getCamEstimate()
        #self.vision.showTargetData()

    def disabledInit(self):
        self.drivetrain.set_motor_stop_modes(to_drive=True, to_break=True, all_motor_override=True, burn_flash=False)
        self.drivetrain.stop_driving()

        self.elevator.motor.set(0)

        self.intake_command_scheduler.cancelAll()

    def disabledPeriodic(self):
        pass

    def autonomousInit(self):
        self.auto_command = self.auto_chooser.getSelected()
        if self.auto_command:
            self.auto_command.schedule()
        else:
            self.drivetrain.reset_pose_estimator(self.drivetrain.get_default_starting_pose())

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        self.table.putNumber("pressedKey", -1)
        self.keys = {0: commands2.cmd.print_("Key 0 pressed"),
                     1: commands2.cmd.print_("Key 1 pressed"),
                     2: commands2.cmd.print_("Key 2 pressed"),
                     3: commands2.cmd.print_("Key 3 pressed"),
                     4: commands2.cmd.print_("Key 4 pressed"),
                     5: commands2.cmd.print_("Key 5 pressed"),
                     6: commands2.cmd.print_("Key 6 pressed"),
                     7: commands2.cmd.print_("Key 7 pressed"),
                     8: commands2.cmd.print_("Key 8 pressed"),
                     9: commands2.cmd.print_("Key 9 pressed"),
                     10: commands2.cmd.print_("Key 10 pressed"),
                     11: commands2.cmd.print_("Key 11 pressed"),
                     12: commands2.cmd.print_("Key 12 pressed"),
                     13: commands2.cmd.print_("Key 13 pressed"),
                     14: commands2.cmd.print_("Key 14 pressed"),
                     -1: commands2.cmd.print_("No key pressed"),}

        self.intake_state_machine.schedule()

        if self.auto_command:
            self.auto_command.cancel()

        self.alliance = "blue"
        if self.drivetrain.flip_to_red_alliance():
            self.alliance = "red"
        self.teleop_auto_command = None

        self.drivetrain.setDefaultCommand(
            DefaultDrive(
                self.drivetrain,
                lambda: wpimath.applyDeadband(-1 * self.driver_controller.getLeftY(), 0.06),
                lambda: wpimath.applyDeadband(-1 * self.driver_controller.getLeftX(), 0.06),
                lambda: wpimath.applyDeadband(-1 * self.driver_controller.getRightX(), 0.1),
                lambda: not self.driver_controller.getRightBumperButton(),
                lambda: self.driver_controller.getLeftBumperButton()
            )
        )

        self.teleop_auto_triggers = {
            "left_reef_align": Trigger(self.driver_controller.getXButtonPressed).onTrue(
                commands2.DeferredCommand(lambda: pathplanToPose(lambda: reef_position_lookup.get(
                    (self.alliance, getCurrentReefZone(self.alliance, self.drivetrain.current_pose), "l"),
                    {}
                ).get("pose", None)))
            ),
             "right_reef_align": Trigger(self.driver_controller.getBButtonPressed).onTrue(
                commands2.DeferredCommand(lambda: pathplanToPose(lambda: reef_position_lookup.get(
                    (self.alliance, getCurrentReefZone(self.alliance, self.drivetrain.current_pose), "r"),
                    {}
                ).get("pose", None)))
             ),
        }

        self.elevator.setDefaultCommand(ElevateManually(
            self.elevator,
            self.arm,
            lambda: (
                wpimath.applyDeadband(self.mech_controller.getLeftY(), 0.2)
            )
        ))

        wpilib.SmartDashboard.putNumber("key_press", -1)

        Trigger(lambda: wpilib.SmartDashboard.getNumber("key_press", -1) == 1).and_(self.isArmSafe).onTrue(
            ElevateToGoal(self.elevator, reef_height_lookup["L1"] + DiverCarlElevatorConsts.kL1OffsetCm)
        )
        Trigger(lambda: wpilib.SmartDashboard.getNumber("key_press", -1) == 2).and_(self.isArmSafe).onTrue(
            ElevateToGoal(self.elevator, reef_height_lookup["L2"] + DiverCarlElevatorConsts.kL2OffsetCm)
        )
        Trigger(lambda: wpilib.SmartDashboard.getNumber("key_press", -1) == 3).and_(self.isArmSafe).onTrue(
            ElevateToGoal(self.elevator, reef_height_lookup["L3"] + DiverCarlElevatorConsts.kL3OffsetCm)
        )
        Trigger(lambda: wpilib.SmartDashboard.getNumber("key_press", -1) == 4).and_(self.isArmSafe).onTrue(
            ElevateToGoal(self.elevator, reef_height_lookup["L4"] + DiverCarlElevatorConsts.kL4OffsetCm)
        )
        Trigger(lambda: wpilib.SmartDashboard.getNumber("key_press", -1) == 5).and_(self.isArmSafe).onTrue(
            ElevateToGoal(self.elevator, DiverCarlElevatorConsts.kChuteHeightCm)
        )

        Trigger(self.mech_controller.getYButtonPressed).onTrue(
            elevCommands.genPivotElevatorCommand(self.arm, self.elevator, PoseOptions.REEF4)
        )
        Trigger(self.mech_controller.getBButtonPressed).onTrue(
            elevCommands.genPivotElevatorCommand(self.arm, self.elevator, PoseOptions.REEF3)
        )
        Trigger(self.mech_controller.getAButtonPressed).onTrue(
            elevCommands.genPivotElevatorCommand(self.arm, self.elevator, PoseOptions.REEF2)
        )
        Trigger(self.mech_controller.getXButtonPressed).onTrue(
            elevCommands.genPivotElevatorCommand(self.arm, self.elevator, PoseOptions.TROUGH)
        )
        Trigger(self.mech_controller.getLeftBumperButtonPressed).onTrue(
            elevCommands.genPivotElevatorCommand(self.arm, self.elevator, PoseOptions.REST)
        )
        Trigger(lambda: abs(wpimath.applyDeadband(self.mech_controller.getLeftY(), 0.2)) > 0).whileTrue(
            ElevateManually(
                self.elevator,
                self.arm,
                lambda: (
                    wpimath.applyDeadband(self.mech_controller.getLeftY(), 0.2)
                )
            )
        )



    def teleopPeriodic(self):
        if self.driver_controller.getLeftTriggerAxis() > 0.5:
            commands2.CommandScheduler.getInstance().cancelAll()
        self.keyPressed = self.table.getNumber("pressedKey", -1)
        self.heartbeat = self.table.getNumber("Stream Deck Heartbeat", 0)
        wpilib.SmartDashboard.putNumber("Stream Deck Life", self.heartbeat)

        wpilib.SmartDashboard.putBoolean("A Button Pressed", self.mech_controller.getRightBumperButton())
        self.intake_command_scheduler.run()

    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self):
        pass

    def getDeployInfo(self, key: str) -> str:
        """Gets the Git SHA of the deployed robot by parsing ~/deploy.json and returning the git-hash from the JSON key OR if deploy.json is unavailable will return "unknown"
            example deploy.json: '{"deploy-host": "DESKTOP-80HA89O", "deploy-user": "ehsra", "deploy-date": "2023-03-02T17:54:14", "code-path": "blah", "git-hash": "3f4e89f138d9d78093bd4869e0cac9b61becd2b9", "git-desc": "3f4e89f-dirty", "git-branch": "fix-recal-nbeasley"}

        Args:
            key (str): The desired json key to get. Popular onces are git-hash, deploy-host, deploy-user

        Returns:
            str: Returns the value of the desired deploy key
        """
        json_object = None
        home = str(Path.home()) + os.path.sep
        releaseFile = home + 'py' + os.path.sep + "deploy.json"
        try:
            # Read from ~/deploy.json
            with open(releaseFile, "r") as openfile:
                json_object = json.load(openfile)
                print(json_object)
                print(type(json_object))
                if key in json_object:
                    return json_object[key]
                else:
                    return f"Key: {key} Not Found in JSON"
        except OSError:
            return "unknown"
        except json.JSONDecodeError:
            return "bad json in deploy file check for unescaped "

    def isArmSafe(self) -> bool:
        """
        """
        return True
