#!/usr/bin/env python3

import wpilib
import wpilib.interfaces
from subsystem import diverCarlElevator

class MyRobot(wpilib.TimedRobot):
    """
    Example robot to test The pull encoder on GPIO 9/8 (A/B) inputs
    """

    def robotInit(self):
        """Robot initialization function"""
        self.elevator = diverCarlElevator.DiverCarlElevator()


    def testInit(self):
        self.elevator.setHeight(0.2)
        wpilib.SmartDashboard.putNumber("Height M", 0.0)

    def testPeriodic(self):
        #self.elevator.setHeight(wpilib.SmartDashboard.getNumber("Height M", 0.0))
        self.elevator.periodic()
        wpilib.SmartDashboard.putNumber("Encoder Pos", self.elevator.getHeightM())
        print("Encoder Pos", self.elevator.getHeightM())
