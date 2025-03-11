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

        self.timer = wpilib.Timer()
        self.timer.start()

    def disabledInit(self):
        self.arm.on_disable()

    def testInit(self):
        wpilib.SmartDashboard.putNumber("Arm Rad", 0.0)
        wpilib.SmartDashboard.putNumber("Arm Deg", 0.0)
        self.arm.on_enable()

    def testPeriodic(self):
        self.arm.periodic()
        wpilib.SmartDashboard.putNumber("Arm Deg", self.arm.getAngleDeg())

        self.arm.on_iteration(self.timer.get())

        # print("Encoder Pos", self.elevator.getPosDeg())
