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
        self.timer = wpilib.Timer()
        self.timer.start()

    def testPeriodic(self):
        if wpilib.SmartDashboard.getBoolean("set", False):
            print("Updating setpoints")
            armArc = wpilib.SmartDashboard.getNumber("Arm Arc", 0.0)
            elevatorCm = wpilib.SmartDashboard.getNumber("Elevator Cm", 0.0)
            print(f"Setting arm to {armArc} and elevator to {elevatorCm}")
            self.elevator.setGoalHeight(elevatorCm)
            self.arm.setArc(armArc)
            self.elevator.resetProfilerState()
            wpilib.SmartDashboard.putBoolean("set", False)


        if(self.timer.advanceIfElapsed(0.020)):
            #run preiodics to make the system move
            if not self.elevator.at_goal:
                self.elevator.goToGoalHeight()
            self.elevator.periodic()
            self.arm.periodic()
