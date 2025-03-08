#!/usr/bin/env python3

import wpilib
import wpilib.interfaces
from subsystem import diverCarlChistera

class MyRobot(wpilib.TimedRobot):
    """
    Example robot to test The pull encoder on GPIO 9/8 (A/B) inputs
    """

    def robotInit(self):
        """Robot initialization function"""
        self.arm = diverCarlChistera.DiverCarlChistera()


    def testInit(self):
        wpilib.SmartDashboard.putNumber("Arm Rad", 0.0)
        wpilib.SmartDashboard.putNumber("Arm Deg", 0.0)

    def testPeriodic(self):
        self.arm.periodic()
        wpilib.SmartDashboard.putNumber("Arm Deg", self.arm.getPosDeg())

        #print("Encoder Pos", self.elevator.getPosDeg())
