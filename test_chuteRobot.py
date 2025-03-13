import wpilib
import wpilib.interfaces
from commands.run_chute import RunChute, RunChuteUntilFree, RunChuteUntilHeld
from subsystem.diverCarlChute import DiverCarlChute as Chute
import commands2
#from data.telemetry import Telemetry

class MyRobot(wpilib.TimedRobot):
    """
    Example robot to test the chute
    """

    def robotInit(self):
        """Robot initialization function"""
        self.chute = Chute()
        # self.telemetry = Telemetry(
        #         self.driver_controller, self.mech_controller, self.drivetrain,
        #         self.elevator, self.chute, wpilib.DriverStation
        #     )
        self.chute.setDefaultCommand(commands2.cmd.run(self.chute.stopChute, self.chute))

    def disabledInit(self):
        pass

    def testInit(self):
        commands2.CommandScheduler.getInstance().cancelAll()

        commands2.cmd.waitSeconds(10).schedule()
        RunChuteUntilHeld(self.chute).schedule()
        commands2.cmd.waitSeconds(2).schedule()
        RunChuteUntilFree(self.chute).schedule()
        commands2.cmd.waitSeconds(10).schedule()
        RunChuteUntilHeld(self.chute, 0.5).schedule()
        commands2.cmd.waitSeconds(2).schedule()
        RunChuteUntilFree(self.chute).schedule()
        commands2.cmd.waitSeconds(10).schedule()
        RunChuteUntilHeld(self.chute, 0.5).schedule()
        commands2.cmd.waitSeconds(2).schedule()
        RunChute(self.chute, True).withTimeout(2).schedule()


    def testPeriodic(self):
        # wpilib.SmartDashboard.putBoolean("A Button Pressed", self.driver_controller.getAButton())
        pass
