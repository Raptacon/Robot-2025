import commands2
from subsystems.diverCarlElevator import DiverCarlElevator


class GrabHatch(commands2.Command):
    def __init__(self, elevator: DiverCarlElevator) -> None:
        super().__init__()
        self.elevator = DiverCarlElevator
        self.addRequirements(self.elevator)

    def initialize(self, heightMeters: float) -> None:
        self.elevator.setHeight(heightMeters)

    def isFinished(self) -> bool:
        return self.elevator.atGoal()
    