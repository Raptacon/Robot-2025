# Native imports
from typing import Callable

# Internal imports
from commands.default_swerve_drive import DefaultDrive
from subsystem.drivetrain.swerve_drivetrain import SwerveDrivetrain

# Third-party imports
import commands2
import wpilib
import wpimath
from commands2.button import Trigger
from pathplannerlib.auto import AutoBuilder

class RobotSwerve:
    """
    Container to hold the main robot code
    """
    def __init__(self, is_disabled: Callable[[], bool]) -> None:
        # Subsystem instantiation
        self.drivetrain = SwerveDrivetrain()

        # HID setup
        self.driver_controller = wpilib.XboxController(0)
        self.mech_controller = wpilib.XboxController(1)

        # Autonomous setup
        self.auto_command = None
        self.auto_chooser = AutoBuilder.buildAutoChooser()
        wpilib.SmartDashboard.putData("Select auto routine", self.auto_chooser)

        Trigger(is_disabled()).debounce(3).onTrue(
            commands2.cmd.runOnce(
                self.drivetrain.set_motor_stop_modes(
                    to_drive=True, to_break=True, all_motor_override=True, burn_flash=True
                ),
                self.drivetrain
            )
        )

    def robotPeriodic(self):
        pass

    def disabledInit(self):
        self.drivetrain.set_motor_stop_modes(to_drive=True, to_break=True, all_motor_override=True, burn_flash=False)
        self.drivetrain.stop_driving()

    def disabledPeriodic(self):
        pass

    def autonomousInit(self):
        self.auto_command = self.auto_chooser.getSelected()
        if self.auto_command:
            self.auto_command.schedule()

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        if self.auto_command:
            self.auto_command.cancel()

        self.drivetrain.setDefaultCommand(
            DefaultDrive(
                self.drivetrain,
                lambda: wpimath.applyDeadband(-1 * self.driver_controller.getLeftY(), 0.06),
                lambda: wpimath.applyDeadband(-1 * self.driver_controller.getLeftX(), 0.06),
                lambda: wpimath.applyDeadband(-1 * self.driver_controller.getRightX(), 0.1),
                True
            )
        )

    def teleopPeriodic(self):
        pass

    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
        pass

    def testPeriodic(self):
        pass
