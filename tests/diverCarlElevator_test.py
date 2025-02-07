from subsystem.diverCarlElevator import DiverCarlElevator
import wpilib.simulation
import rev
from wpimath.system.plant import DCMotor


def test_elevatorSubsystem() -> None:
    simTime = 0.05
    elevator = DiverCarlElevator(simTime)
    simPrimaryMotor = rev.SparkSim(elevator._primaryMotor, DCMotor(12, 10, 10, 10, 5000, 1))
    simPfollowerMotor = rev.SparkSim(elevator._followerMotor, DCMotor(12, 10, 10, 10, 5000, 1))


    #test some contsturction time variables
    assert elevator._dt == simTime
    assert elevator._controller.getPeriod() == simTime


    # test the nominal operations by setting the elevator to 1m and then 0m
    # call periodic until at goal and increase the encoder reading. If encoder reading is < 0m or >2m, fail test so we don't get "stuck"

    assert elevator.getHeight() == 0
    elevator.setHeight(1)
    assert elevator.getHeight() == 0
    assert elevator.atGoal() == False
    encSim = wpilib.simulation.EncoderSim(elevator._encoder)
    simPos = 0
    while not elevator.atGoal():
        elevator.periodic()
        encSim.setDistance(simPos)
        simPos += 0.0001 #0.1mm at a time
        #print(f"SimPos: {simPos}, encoder distance: {elevator.encoder.getDistance()} error: {elevator.getError()}")
        if(simPos > 2):
            raise("Elevator did not reach goal")
        
        assert elevator._limitAlert.get() == False
        assert elevator._trackingAlert.get() == True
        assert "Elevator at" in elevator._trackingAlert.getText()
        
    #now that we are at goal run one more cycle to verify behaviors
    elevator.periodic()
    assert elevator._limitAlert.get() == False
    assert elevator._trackingAlert.get() == True
    assert "Elevator stable" in elevator._trackingAlert.getText()

    assert elevator.atGoal() == True
    assert elevator.getError() < 0.01
    elevator.setHeight(0)
    assert elevator.atGoal() == False
    while not elevator.atGoal():
        elevator.periodic()
        if simPos > 0.0:
            encSim.setDistance(simPos)
        simPos -= 0.0001 #0.1mm at a time
        if simPos < -2:
            raise("Elevator did not reach goal")


    assert elevator.atGoal() == True
    assert elevator.getError() < 0.01

    #now that we are at goal run one more cycle to verify behaviors
    elevator.periodic()
    assert elevator._limitAlert.get() == False
    assert elevator._trackingAlert.get() == True
    assert "Elevator stable" in elevator._trackingAlert.getText()    

    #test limit switches
    #set elevator at 0m
    #then set elevator to move to 1m.
    #simulate forward limit being hit
    #release forward limit and verify motors restart

    encSim.setDistance(0)
    elevator.setHeight(1.0)
    simPrimaryMotor.getForwardLimitSwitchSim().setPressed(True)
    elevator._motors.set(1.0)
    elevator.periodic()
    assert elevator._motors.get() == 0
    assert elevator._limitAlert.get() == True
    simPrimaryMotor.getForwardLimitSwitchSim().setPressed(False)
    elevator.periodic()
    assert elevator._motors.get() > 0
    assert elevator._limitAlert.get() == False

    #test disable while setup for it
    elevator.disable()
    elevator.periodic()
    assert elevator._motors.get() == 0
    assert elevator._disabled == True

    #test reverse limit now
    #set elevator at 1m
    #then set elevator to move to 0.
    #simulate reverse limit being hit
    #release reverse limit and verify motors restart

    encSim.setDistance(1.0)
    elevator.setHeight(0.0)
    assert elevator._disabled == False

    
    simPrimaryMotor.getReverseLimitSwitchSim().setPressed(True)
    elevator._motors.set(1.0)
    elevator.periodic()
    assert elevator._motors.get() == 0
    assert elevator._limitAlert.get() == True
    simPrimaryMotor.getReverseLimitSwitchSim().setPressed(False)
    elevator.periodic()
    assert elevator._motors.get() < 0
    assert elevator._limitAlert.get() == False


    #test incremental move and stop move
    elevator.setHeight(0.5)
    assert elevator.getSetHeight() == 0.5
    elevator.setIncrementalMove(0.1)
    assert elevator.getSetHeight() == 0.6
    encSim.setDistance(0.55)
    elevator.stopElevator()
    assert elevator.getSetHeight() == 0.55

    print("Elevator test passed")


def test_elevatorCommand():
    pass