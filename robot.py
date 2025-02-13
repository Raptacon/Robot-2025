#!/usr/bin/env python3

import wpilib


class MyRobot(wpilib.TimedRobot):
    """
    Our default robot class, pass it to wpilib.run
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of (self) -> None:
    which runs the scheduler for you
    """
    #def __init__(self) -> None:
     #   pass

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        self.analog_input = wpilib.AnalogInput(3)

    def robotPeriodic(self) -> None:
        averaged_voltage = self.analog_input.getAverageVoltage()

        wpilib.SmartDashboard.putNumber("Air Sensor Voltage", averaged_voltage)

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        #self.container.disabledInit()

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        #self.container.disabledPeriodic()

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        #self.container.autonomousInit()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        #self.container.autonomousPeriodic()
        pass

    def teleopInit(self) -> None:
        #self.container.teleopInit()
        pass
    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        #self.container.teleopPeriodic()
        pass
    def testInit(self) -> None:
        #self.container.testInit()
        pass
    def testPeriodic(self) -> None:
        #self.container.testPeriodic()
        pass

if __name__ == "__main__":
    print("Please run python -m robotpy <args>")
    exit(1)
