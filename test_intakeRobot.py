import wpilib
import wpilib.interfaces
from subsystem.captainIntake import CaptainIntake, CaptainIntakeStateMachine
import commands2

class MyRobot(wpilib.TimedRobot):
    """
    Example robot to test the intake
    """

    def robotInit(self):
        """Robot initialization function"""
        self.intake = CaptainIntake()
        self.state_machine = CaptainIntakeStateMachine(self.intake)
        self.command_scheduler = commands2.CommandScheduler.getInstance()

        self.driver_controller = wpilib.XboxController(0)

    def disabledInit(self):
        # commands2.CommandScheduler.getInstance().cancelAll()
        self.command_scheduler.cancelAll()
        # self.command_scheduler.cancel(self.state_machine)

    def testInit(self):
        self.state_machine.schedule()

    def testPeriodic(self):
        wpilib.SmartDashboard.putBoolean("A Button Pressed", self.driver_controller.getAButton())
        self.command_scheduler.run()
