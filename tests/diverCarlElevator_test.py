from subsystem import diverCarlElevator
from subsystem import diverCarlChistera
import pytest
import wpilib.simulation
import rev
from wpimath.system.plant import DCMotor
from robot import MyRobot

def test_elevatorSubsystemSafety(robot : MyRobot) -> None:
    robot = robot.getRobot()
    elevator = robot.elevator
    arm = robot.arm

    simArmMotor = rev.SparkSim(arm._primaryMotor, DCMotor(12, 10, 10, 10, 5000, 1))
    simElevMotor = rev.SparkSim(elevator.motor, DCMotor(12, 10, 10, 10, 5000, 1))
    simArmEncoder = simArmMotor.getRelativeEncoderSim()
    simElevEncoder = simElevMotor.getRelativeEncoderSim()

    #verify operation with no arm
    elevator.setGoalHeight(0.0)
    simElevEncoder.setPosition(0)
    elevator.periodic()
    assert simElevMotor.getClosedLoopSlot() == rev.ClosedLoopSlot.kSlot0
    assert simElevMotor.getSetpoint() == pytest.approx(0.0)
