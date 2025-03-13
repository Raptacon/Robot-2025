# Internal imports
from subsystem.diverCarlChute import DiverCarlChute

# Third-Party Imports
import commands2
from wpilib import Timer


class RunChute(commands2.Command):
    """
    """
    def __init__(self, chute: DiverCarlChute, reverse: bool = False) -> None:
        """
        """
        super().__init__()
        self.chute = chute
        self.reverse = reverse
        self.addRequirements(self.chute)

    def execute(self) -> None:
        """
        """
        self.chute.runChute(reverse=self.reverse)

    def end(self, interrupted: bool) -> None:
        """
        """
        self.chute.stopChute()

    def isFinished(self) -> bool:
        """
        """
        return False


class RunChuteUntilHeld(commands2.Command):
    """
    """
    def __init__(self, chute: DiverCarlChute, hold_duration_seconds: float = 0.0) -> None:
        """
        """
        super().__init__()
        self.chute = chute
        self.hold_duration_seconds = hold_duration_seconds
        self.beam_broken_in_execution = False
        self.timer = Timer()
        self.addRequirements(self.chute)

    def execute(self) -> None:
        """
        """
        self.chute.runChute()
        if self.chute.breakbeam_broken and (not self.beam_broken_in_execution):
            self.beam_broken_in_execution = True
            self.timer.restart()

    def end(self, interrupted: bool) -> None:
        """
        """
        self.chute.stopChute()

    def isFinished(self) -> bool:
        """
        """
        return self.beam_broken_in_execution and (self.timer.get() > self.hold_duration_seconds)


class RunChuteUntilFree(commands2.Command):
    """
    """
    def __init__(self, chute: DiverCarlChute) -> None:
        """
        """
        super().__init__()
        self.chute = chute
        self.addRequirements(self.chute)

    def execute(self) -> None:
        """
        """
        self.chute.runChute()

    def end(self, interrupted: bool) -> None:
        """
        """
        self.chute.stopChute()

    def isFinished(self) -> bool:
        """
        """
        return not self.chute.breakbeam_broken
