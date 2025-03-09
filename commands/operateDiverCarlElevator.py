# Internal imports
from subsystem.diverCarlElevator import DiverCarlElevator

# Third-Party Imports
import commands2


class OperateElevator(commands2.Command):
    def __init__(self, elevator: DiverCarlElevator, level: str) -> None:
        super().__init__()
        self.elevator = elevator
        self.level_height = elevator.known_positions.get(level, 0)
        self.addRequirements(self.elevator)

    def initialize(self) -> None:
        self.elevator.setHeight(50)

    def isFinished(self) -> bool:
        return self.elevator.atGoal()
