import wpilib
import wpilib.interfaces
from subsystem.captainIntake import CaptainIntake


class MyRobot(wpilib.TimedRobot):
    """
    Example robot to test the intake
    """

    def robotInit(self):
        """Robot initialization function"""
        self.intake_state_machines = CaptainIntake()

        self.driver_controller = wpilib.XboxController(0)

        self.timer = wpilib.Timer()
        self.timer.start()

    def disabledInit(self):
        self.intake_state_machines.on_disable()

    def testInit(self):
        self.intake_state_machines.on_enable()

    def testPeriodic(self):
        wpilib.SmartDashboard.putBoolean("A Button Pressed", self.driver_controller.getAButton())
        self.intake_state_machines.on_iteration(self.timer.get())