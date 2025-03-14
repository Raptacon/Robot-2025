# Native imports
from typing import Callable

# Internal imports
from subsystem.diverCarlElevator import DiverCarlElevator
from subsystem.diverCarlChistera import DiverCarlChistera
from constants import MechConsts as mc, PoseOptions as p_opt

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
        pivot: DiverCarlChistera,
        goal_height_increment_cm: float,
    ) -> None:
        """
        """
        super().__init__()
        self.elevator = elevator
        self.pivot = pivot
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
        pivot: DiverCarlChistera,
        velocity_percentaage: Callable[[], float],
    ) -> None:
        """
        """
        super().__init__()
        self.elevator = elevator
        self.pivot = pivot
        self.velocity_percentaage = velocity_percentaage
        self.addRequirements(self.elevator, self.pivot)

    def execute(self):
        """
        """
        if abs(self.velocity_percentaage()) > 0.25:
            self.pivot.setArc((mc.kArmSafeAngleStart + mc.kArmSafeAngleEnd) / 2)
        else:
            self.pivot.setArc(0)

        self.elevator.manualControl(-1 * self.velocity_percentaage())

    def isFinished(self) -> bool:
        """
        """
        return False

class PivotToGoal(commands2.Command):
    def __init__(self,
                 pivot: DiverCarlChistera,
                 goal_arc: float):
        super().__init__()
        self.pivot = pivot
        self.goal_arc = goal_arc
        self.addRequirements(self.pivot)
    
    def initialize(self):
        self.pivot.setArc(self.goal_arc)
    
    def isFinished(self) -> bool:
        self.pivot.atGoal()

class PivotToIncrementedGoal(commands2.Command):
    def __init__(self,
                 pivot: DiverCarlChistera,
                 goal_arc_increment: float):
        self.pivot = pivot
        self.goal_arc_increment = goal_arc_increment
        self.addRequirements(self.pivot)
    
    def initialize(self):
        self.pivot.setArc(self.pivot.getSetArc() + self.goal_arc_increment)
    
    def isFinished(self):
        self.pivot.atGoal()
    
def genPivotElevatorCommand(pivot: DiverCarlChistera,
                            elevator: DiverCarlElevator,
                            reef_opt: p_opt = -1,
                            manual_goal_height_cm = None,
                            manual_goal_arc = None):
    """
    Takes a pivot, elevator, and reef_opt for a fixed preset.
    If reef_opt is MANUAL (or other unrecognized value),
        custom goal height/arc can be given.
    """
    match reef_opt:
        case p_opt.REST:
            goal_height_cm, goal_arc = 0, 0
        case p_opt.TROUGH:
            goal_height_cm, goal_arc = mc.kElevatorTrough, mc.kArmAngleTrough
        case p_opt.REEF2:
            goal_height_cm, goal_arc = mc.kElevatorReef2, mc.kArmAngleReef2
        case p_opt.REEF3:
            goal_height_cm, goal_arc = mc.kElevatorReef3, mc.kArmAngleReef3
        case p_opt.REEF4:
            goal_height_cm, goal_arc = mc.kElevatorReef4, mc.kArmAngleReef4
        case _:
            goal_height_cm, goal_arc = manual_goal_height_cm or 0, manual_goal_arc or 0
    return commands2.ParallelCommandGroup(
        PivotToGoal(pivot, goal_arc),
        ElevateToGoal(elevator, goal_height_cm)
    )
