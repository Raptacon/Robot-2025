#!/usr/bin/env python3

import wpilib
import wpilib.interfaces
from subsystem import diverCarlElevator
from subsystem import diverCarlChistera

class MyRobot(wpilib.TimedRobot):
    """
    Example robot to test The pull encoder on GPIO 9/8 (A/B) inputs
    """

    def robotInit(self):
        """Robot initialization function"""
        self.elevator = diverCarlElevator.DiverCarlElevator()
        self.arm = diverCarlChistera.DiverCarlChistera()
        self.elevator.setArm(self.arm)
        self.arm.setElevator(self.elevator)


    def testInit(self):
        wpilib.SmartDashboard.putNumber("Elevator Cm", 0.0)
        wpilib.SmartDashboard.putNumber("Arm Arc", 0.0)
        wpilib.SmartDashboard.putBoolean("set", False)

    def testPeriodic(self):
        if wpilib.SmartDashboard.getBoolean("set", False):
            print("Updating setpoints")
            self.elevator.setGoalHeight(wpilib.SmartDashboard.getNumber("Elevator Cm", 0.0))
            self.arm.setArc(wpilib.SmartDashboard.getNumber("Arm Arc", 0.0))
            wpilib.SmartDashboard.putBoolean("set", False)

        #run preiodics to make the system move
        self.elevator.goToGoalHeight()
        self.elevator.periodic()
        self.arm.periodic()
