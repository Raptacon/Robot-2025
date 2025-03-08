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
        #self.elevator.setHeight(0.2)
        wpilib.SmartDashboard.putNumber("Height CM", 0.0)

    def testPeriodic(self):
        self.elevator.setHeight(wpilib.SmartDashboard.getNumber("Height CM", 0.0))
        self.elevator.periodic()
        wpilib.SmartDashboard.putNumber("Height Cm", self.elevator.getHeightCm())
        wpilib.SmartDashboard.putNumber("Encoder Pos", self.elevator._encoder.getPosition())
        wpilib.SmartDashboard.putBoolean("At Goal", self.elevator.atGoal())
        wpilib.SmartDashboard.putBoolean("Lower Limit", self.elevator.getReverseLimit())
        wpilib.SmartDashboard.putBoolean("Upper Limit", self.elevator.getForwardLimit())
        wpilib.SmartDashboard.putNumber("Vel", self.elevator._encoder.getVelocity())
        wpilib.SmartDashboard.putNumber("Error", self.elevator.getError())

        #print(self.elevator._primaryMotor.PeriodicStatus0().softForwardLimitReached)
