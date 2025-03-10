# Native imports
from typing import Callable

# Internal imports
from subsystem.diverCarlElevator import DiverCarlElevator

# Third-Party Imports
import commands2


class ElevateToGoal(commands2.Command):
    """
    """
    def __init__(
        self,
        elevator: DiverCarlElevator,
        goal_height_cm: float,
    ) -> None:
        """
        """
        super().__init__()
        self.elevator = elevator
        self.goal_height_cm = goal_height_cm
        self.addRequirements(self.elevator)

    def initialize(self):
        """
        """
        self.elevator.resetProfilerState()
        self.elevator.setGoalHeight(self.goal_height_cm)

    def execute(self):
        """
        """
        self.elevator.goToGoalHeight()

    def isFinished(self) -> bool:
        """
        """
        return self.elevator.checkIfAtGoalHeight()


class ElevateToIncrementedGoal(commands2.Command):
    """
    """
    def __init__(
        self,
        elevator: DiverCarlElevator,
        goal_height_increment_cm: float,
    ) -> None:
        """
        """
        super().__init__()
        self.elevator = elevator
        self.goal_height_increment_cm = goal_height_increment_cm
        self.addRequirements(self.elevator)

    def initialize(self):
        """
        """
        self.elevator.resetProfilerState()
        self.elevator.incrementGoalHeight(self.goal_height_increment_cm)

    def execute(self):
        """
        """
        self.elevator.goToGoalHeight()

    def isFinished(self) -> bool:
        """
        """
        return self.elevator.checkIfAtGoalHeight()


class ElevateManually(commands2.Command):
    """
    """
    def __init__(
        self,
        elevator: DiverCarlElevator,
        velocity_percentaage: Callable[[], float],
    ) -> None:
        """
        """
        super().__init__()
        self.elevator = elevator
        self.velocity_percentaage = velocity_percentaage
        self.addRequirements(self.elevator)

    def execute(self):
        """
        """
        self.elevator.manualControl(self.velocity_percentaage())

    def isFinished(self) -> bool:
        """
        """
        return False
