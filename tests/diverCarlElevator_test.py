from subsystem.diverCarlElevator import DiverCarlElevator
import wpilib.simulation


def test_elevator() -> None:
    simTime = 0.05
    elevator = DiverCarlElevator(simTime)
    assert elevator.dt == simTime
    assert elevator.controller.getPeriod() == simTime
    assert elevator.getHeight() == 0
    elevator.setHeight(1)
    assert elevator.getHeight() == 0
    assert elevator.atGoal() == False
    encSim = wpilib.simulation.EncoderSim(elevator.encoder)
    simPos = 0
    while not elevator.atGoal():
        elevator.periodic()
        encSim.setDistance(simPos)
        simPos += 0.0001 #0.1mm at a time
        #print(f"SimPos: {simPos}, encoder distance: {elevator.encoder.getDistance()} error: {elevator.getError()}")
        if(simPos > 2):
            raise("Elevator did not reach goal")
        
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
    print("Elevator test passed")