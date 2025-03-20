import wpilib
import wpimath
from subsystem.captainIntake import CaptainIntake
import commands.operate_intake as IntakeCommands
import commands2

class MyRobot(wpilib.TimedRobot):
    """
    Example robot to test the intake
    """

    def robotInit(self):
        """Robot initialization function"""
        self.intake = CaptainIntake()

        self.driver_controller = wpilib.XboxController(0)
        self.mech_controller = wpilib.XboxController(1)

    def disabledInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()
        commands2.cmd.sequence(
            IntakeCommands.IntakeToFront(self.intake, holdDurationSeconds=0.1, reverse=False),
            commands2.cmd.waitSeconds(1),
            IntakeCommands.IntakeToFrontOnly(self.intake),
            commands2.cmd.waitSeconds(0.5),
            IntakeCommands.IntakeToBack(self.intake, holdDurationSeconds=0.1, reverse=True),
            commands2.cmd.waitSeconds(1),
            IntakeCommands.IntakeToFrontOnly(self.intake),
            commands2.cmd.waitSeconds(0.5),
            IntakeCommands.generateIntakeMaintainHold(self.intake),
            commands2.cmd.waitSeconds(1),
            IntakeCommands.generateIntakeFrontForwardHold(self.intake),
            commands2.cmd.waitSeconds(1)
        ).schedule()
        self.intake.setDefaultCommand(IntakeCommands.IntakeManually(
            lambda: wpimath.applyDeadband(self.mech_controller.getRightTriggerAxis(), 0.1),
            self.intake
        ))

    def testPeriodic(self):
        pass
