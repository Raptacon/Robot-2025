# Native imports
from typing import Callable

# Internal imports
from subsystem.diverCarlElevator import DiverCarlElevator

# Third-Party Imports
import commands2


class OperateElevator(commands2.Command):
    """
    """
    def __init__(
        self,
        elevator: DiverCarlElevator,
        goal_height_cm: Callable[[], float],
        terminate_at_goal: bool = False
    ) -> None:
        """
        """
        super().__init__()
        self.elevator = elevator
        self.goal_height_cm = goal_height_cm
        self.terminate_at_goal = terminate_at_goal
        self.addRequirements(self.elevator)

    def execute(self):
        """
        """
        self.elevator.setGoalHeight(self.goal_height_cm())

    def isFinished(self) -> bool:
        """
        """
        if self.terminate_at_goal:
            return self.elevator.at_goal
        return False
