import wpilib
import wpimath
from subsystem.captainIntake import CaptainIntake
import commands.operate_intake as IntakeCommands
import commands2


class MyRobot(commands2.TimedCommandRobot):
    """
    Example robot to test the intake
    """

    def robotInit(self):
        """Robot initialization function"""
        self.intake = CaptainIntake()

        self.driver_controller = wpilib.XboxController(0)
        self.mech_controller = wpilib.XboxController(1)

    def teleopInit(self):
        commands2.cmd.sequence(
            commands2.DeferredCommand(
                IntakeCommands.IntakeToFront(self.intake, holdDurationSeconds=0.1, reverse=False),
                self.intake
            ),
            commands2.cmd.waitSeconds(1),
            commands2.DeferredCommand(
                IntakeCommands.IntakeToFrontOnly(self.intake), self.intake
            ),
            commands2.cmd.waitSeconds(0.5),
            commands2.DeferredCommand(
                IntakeCommands.IntakeToBack(self.intake, holdDurationSeconds=0.1, reverse=True), self.intake
            ),
            commands2.cmd.waitSeconds(1),
            commands2.DeferredCommand(
                IntakeCommands.IntakeToFrontOnly(self.intake), self.intake
            ),
            commands2.cmd.waitSeconds(0.5),
            commands2.DeferredCommand(
                IntakeCommands.generateIntakeMaintainHold(self.intake), self.intake
            ),
            commands2.cmd.waitSeconds(1),
            commands2.DeferredCommand(
                IntakeCommands.generateIntakeFrontForwardHold(self.intake), self.intake
            ),
            commands2.cmd.waitSeconds(1)
        ).schedule()
        self.intake.setDefaultCommand(IntakeCommands.IntakeManually(
            lambda: wpimath.applyDeadband(self.mech_controller.getRightTriggerAxis(), 0.1),
            self.intake
        ))

    def disabledInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self):
        pass
