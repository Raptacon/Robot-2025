#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import wpilib.interfaces


class MyRobot(wpilib.TimedRobot):
    """
    This is a demo program showing the use of the DifferentialDrive class.
    Runs the motors with tank steering.
    """

    def robotInit(self):
        """Robot initialization function"""
        pass

    def testInit(self):
        wpilib.SmartDashboard.putNumber("Encoder Pos", 0)
        self.wireEncoder = wpilib.Encoder(9, 8, False, wpilib.interfaces._interfaces.CounterBase.EncodingType.k1X)
        self.wireEncoder.setDistancePerPulse(0.1)
        #self.wireEncoder.setIndexSource(7, wpilib.Encoder.IndexingType.kResetOnRisingEdge);
        self.wireReset = wpilib.DigitalInput(7)
        #self.wireRestTrig = False;

    def testPeriodic(self):
        wpilib.SmartDashboard.putNumber("Encoder Pos", self.wireEncoder.getDistance())
        #wpilib.SmartDashboard.putBoolean("Encoder Reset", self.wireReset.get())
        print(f"Encoder Pos: {self.wireEncoder.getDistance():4.02f} Reset: {self.wireReset.get()}")
