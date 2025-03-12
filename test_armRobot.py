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
        wpilib.SmartDashboard.putNumber("set arc", 0.0)
        wpilib.SmartDashboard.putNumber("curr arc", 0.0)

    def testPeriodic(self):
        self.arm.setArc(wpilib.SmartDashboard.getNumber("set arc", 0.0))
        self.arm.periodic()
        wpilib.SmartDashboard.putNumber("curr arc", self.arm.getArc())

        #print("Encoder Pos", self.elevator.getPosDeg())