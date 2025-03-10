from subsystem.diverCarlElevator import DiverCarlElevator
import wpilib.simulation
import rev
from wpimath.system.plant import DCMotor

def test_elevatorSubsystem() -> None:
    simTime = 0.06
    elevator = DiverCarlElevator(update_period=simTime)
    simPrimaryMotor = rev.SparkSim(elevator.motor, DCMotor(12, 10, 10, 10, 5000, 1))

    # test some consturction time variables
    assert elevator.update_period == simTime

    # test the nominal operations by setting the elevator to 1m and then 0m
    # call periodic until at goal and increase the encoder reading.
    # If encoder reading is < 0m or >2m, fail test so we don't get "stuck"

    assert elevator.current_height_above_zero == 0
    elevator.setGoalHeight(100)
    assert elevator.current_height_above_zero == 0
    assert elevator.at_goal == False
    encSim = wpilib.simulation.EncoderSim(elevator.encoder)
    simPos = 0
    while not elevator.at_goal:
        elevator.periodic()
        encSim.setDistance(simPos)
        simPos += 0.01  #0.1mm at a time
        if simPos > 150:
            raise("Elevator did not reach goal")

    #  now that we are at goal run one more cycle to verify behaviors
    elevator.periodic()

    assert elevator.at_goal == True
    assert elevator.error_from_goal < 1
    elevator.setGoalHeight(0)
    assert elevator.at_goal == False
    while not elevator.at_goal():
        elevator.periodic()
        if simPos > 0.0:
            encSim.setDistance(simPos)
        simPos -= 0.01  #0.1mm at a time
        if simPos < -150:
            raise Exception("Elevator did not reach goal")

    assert elevator.at_goal == True
    assert elevator.error_from_goal < 1

    # now that we are at goal run one more cycle to verify behaviors
    elevator.periodic()

    # test limit switches
    # set elevator at 0m
    # then set elevator to move to 1m.
    # simulate forward limit being hit
    # release forward limit and verify motors restart

    encSim.setDistance(0)
    elevator.setGoalHeight(100)
    simPrimaryMotor.getForwardLimitSwitchSim().setPressed(True)
    elevator.motor.set(1.0)
    elevator.periodic()
    assert elevator.motor.get() == 0
    assert elevator.at_top_limit == True
    simPrimaryMotor.getForwardLimitSwitchSim().setPressed(False)
    elevator.goToGoalHeight()
    elevator.periodic()
    assert elevator.motor.get() > 0
    assert elevator.at_top_limit == False

    # test disable while setup for it
    elevator.disable()
    elevator.goToGoalHeight()
    elevator.periodic()
    assert elevator.motor.get() == 0

    # test reverse limit now
    # set elevator at 1m
    # then set elevator to move to 0.
    # simulate reverse limit being hit
    # release reverse limit and verify motors restart

    encSim.setDistance(100)
    elevator.setGoalHeight(0.0)

    simPrimaryMotor.getReverseLimitSwitchSim().setPressed(True)
    elevator.motor.set(-1.0)
    encSim.setDistance(100)
    elevator.periodic()
    assert elevator.motor.get() == 0
    assert elevator.encoder.getPosition() == 0
    assert elevator.at_bottom_limit == True
    simPrimaryMotor.getReverseLimitSwitchSim().setPressed(False)
    encSim.setDistance(100)
    elevator.goToGoalHeight()
    elevator.periodic()
    assert elevator.motor.get() < 0
    assert elevator.at_bottom_limit == False

    # test incremental move and stop move
    elevator.setGoalHeight(50)
    assert elevator.current_goal_height == 50
    elevator.incrementGoalHeight(10)
    assert elevator.current_goal_height == 60
    encSim.setDistance(55)
    elevator.motor.set(0)
    assert elevator.current_goal_height == 55


    # test robot mechnical to set adjustment
    # elevator.setGoalHeight(0)
    # assert elevator.current_goal_height == DiverCarlElevatorConsts.kHeightAtZeroCm
    # assert elevator._curentGoal == 0
    # elevator.setHeight(DiverCarlElevatorConsts.kMechDeltaHeightM)
    # assert elevator.getSetHeightM() == DiverCarlElevatorConsts.kMechDeltaHeightM
    # assert elevator._curentGoal == 0
    # elevator.setHeight(DiverCarlElevatorConsts.kMechDeltaHeightM + 0.1)
    # assert elevator.getSetHeightM() == DiverCarlElevatorConsts.kMechDeltaHeightM + 0.1
    # assert math.isclose(elevator._curentGoal, 0.1)


    print("Elevator test passed")


def test_elevatorCommand():
    pass
