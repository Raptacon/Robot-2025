# Internal imports
from commands.default_swerve_drive import DefaultDrive
from subsystem.drivetrain.drivetrain import SwerveDrivetrain

# Third-party imports
import commands2
import wpilib
import wpimath
from commands2.button import Trigger
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto

class robotswerve:
    """
    Container to hold the main robot code
    """

    def __init__(self) -> None:
        self.drivetrain = SwerveDrivetrain()
        self.driver_controller = wpilib.XboxController(0)
        self.auto_command = None
        #self.auto_chooser = AutoBuilder.buildAutoChooser()
        #wpilib.SmartDashboard.putData("Select auto routine", self.auto_chooser)

        Trigger(self.isDisabled).debounce(3).onTrue(
            commands2.cmd.runOnce(
                self.drivetrain.set_motor_stop_modes(
                    to_drive=True, to_break=True, all_motor_override=True, burn_flash=True
                ),
                self.drivetrain
            )
        )

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

    def disabledInit(self):
        self.drivetrain.set_motor_stop_modes(to_drive=True, to_break=True, all_motor_override=True, burn_flash=False)
        self.drivetrain.stop_driving()

    def disabledPeriodic(self):
        pass

    def autonomousInit(self):
        pass
        #self.drivetrain.set_motor_stop_modes(to_drive=True, to_break=True, all_motor_override=True, burn_flash=False)
        #path = PathPlannerPath.fromPathFile("3_angular_only")
        #self.drivetrain.setDefaultCommand(AutoBuilder.followPath(path))
        #self.auto_command = PathPlannerAuto('11M_slalom_perpen_flip')
        # if self.auto_command:
        #     self.auto_command.schedule()

        #auto_routine = self.auto_chooser.getSelected()
        #if auto_routine:
            #auto_routine.schedule()

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        if self.auto_command:
            self.auto_command.cancel()
        #self.drivetrain.set_motor_stop_modes(to_drive=True, to_break=True)
        #self.drivetrain.set_motor_stop_modes(to_drive=False, to_break=False)
        self.drivetrain.setDefaultCommand(
            DefaultDrive(
                self.drivetrain,
                lambda: wpimath.applyDeadband(-1 * self.driver_controller.getLeftY(), 0.06),
                lambda: wpimath.applyDeadband(-1 * self.driver_controller.getLeftX(), 0.06),
                lambda: wpimath.applyDeadband(-1 * self.driver_controller.getRightX(), 0.1),
                False
            )
        )

    def teleopPeriodic(self):
        pass

    def testInit(self):
        pass

    def testPeriodic(self):
        pass
