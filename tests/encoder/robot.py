#!/usr/bin/env python3

import wpilib
import wpilib.interfaces


class MyRobot(wpilib.TimedRobot):
    """
    Example robot to test The pull encoder on GPIO 9/8 (A/B) inputs
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
