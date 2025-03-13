import rev
from wpimath.system.plant import DCMotor
import math
import pytest
from constants import MechConsts as mc

from robot import MyRobot

def test_elevatorSubsystemSafety(robot : MyRobot) -> None:
    robot = robot.getRobot()
    elevator = robot.elevator
    arm = robot.arm
    simArmMotor = rev.SparkSim(arm._primaryMotor, DCMotor(12, 10, 10, 10, 5000, 1))
    simElevMotor = rev.SparkSim(elevator.motor, DCMotor(12, 10, 10, 10, 5000, 1))

    #simArmEncoder = simArmMotor.getRelativeEncoderSim()
    simElevEncoder = simElevMotor.getRelativeEncoderSim()

    #verify operation with no elevator
    arm.setArc(1.0)
    arm.periodic()
    assert simArmMotor.getClosedLoopSlot() == rev.ClosedLoopSlot.kSlot0
    assert math.isclose(simArmMotor.getSetpoint(), -mc.kArmSafeAngleEnd, abs_tol=0.0001)

    #attach the elevator
    arm.setElevator(elevator)
    simElevEncoder.setPosition(0)

    #test at 3 safe heights so value is between 0 and max safe
    for height in [0, mc.kElevatorSafeHeight * 0.999 , mc.kElevatorSafeHeight]:
        simElevEncoder.setPosition(height)
        arm.setArc(1.0)
        arm.periodic()
        assert simArmMotor.getSetpoint() == pytest.approx(-mc.kArmSafeAngleEnd)

        #min arm angle
        arm.setArc(0)
        arm.periodic()
        assert simArmMotor.getSetpoint() == pytest.approx(0.0)

    #test min arm and max angle when arm is up. Note arm max is enforced with softlimits so will simulate with max == 1
    for height in [mc.kElevatorSafeHeight * 1.001, mc.kElevatorSafeHeight * 2]:
        simElevEncoder.setPosition(height)
        arm.setArc(1.0)
        arm.periodic()
        assert simArmMotor.getSetpoint() == pytest.approx(-1.0)

        #min arm angle
        arm.setArc(0)
        arm.periodic()
        assert simArmMotor.getSetpoint() == pytest.approx(-mc.kArmSafeAngleStart)
