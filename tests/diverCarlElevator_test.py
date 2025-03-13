import pytest
import rev
from wpimath.system.plant import DCMotor
from robot import MyRobot

from constants import MechConsts as mc
from subsystem.diverCarlElevator import DiverCarlElevator
from subsystem.diverCarlChistera import DiverCarlChistera

def test_elevatorSubsystemSafety(robot : MyRobot) -> None:
    robot = robot.getRobot()
    elevator: DiverCarlElevator = robot.elevator
    arm: DiverCarlChistera = robot.arm


    simArmMotor = rev.SparkSim(arm._primaryMotor, DCMotor(12, 10, 10, 10, 5000, 1))
    simElevMotor = rev.SparkSim(elevator.motor, DCMotor(12, 10, 10, 10, 5000, 1))
    simArmEncoder = simArmMotor.getRelativeEncoderSim()
    simElevEncoder = simElevMotor.getRelativeEncoderSim()

    elevator.setArm(None)
    #verify operation with no arm
    elevator.setGoalHeight(0.0)
    simElevEncoder.setPosition(0)
    elevator.goToGoalHeight()
    assert simElevMotor.getClosedLoopSlot() == rev.ClosedLoopSlot.kSlot0
    assert simElevMotor.getSetpoint() == pytest.approx(0.0)

    elevator.setArm(arm)

    #test arm in unsafe positions
    for pos in [0,  mc.kArmSafeAngleStart*0.999]:

        simArmEncoder.setPosition(-pos)
        elevator.setGoalHeight(100)
        simElevEncoder.setPosition(100)

        #arb loop count, but make sure value does not converge past safe height
        for i in range(1000):
            elevator.goToGoalHeight()
            elevator.periodic()
            assert simElevMotor.getSetpoint() <= mc.kElevatorSafeHeight
            #move elevator to last position
            simElevEncoder.setPosition(elevator.last_profiler_state.position)

    #test arm in safe positions
    # note float error, adding 0.01 to start
    for pos in [mc.kArmSafeAngleStart+0.01, mc.kArmSafeAngleEnd, 1.0]:
        testHeight = 100
        simArmEncoder.setPosition(-pos)
        simElevEncoder.setPosition(0)
        elevator.setGoalHeight(testHeight)

        #arb loop count, but make sure value does not converge past safe height
        for i in range(1000):
            elevator.goToGoalHeight()
            elevator.periodic()
            #move elevator to last position
            simElevEncoder.setPosition(elevator.last_profiler_state.position)

        assert simElevMotor.getSetpoint() == pytest.approx(testHeight-elevator.height_at_zero)
