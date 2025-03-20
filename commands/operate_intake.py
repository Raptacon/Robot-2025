from typing import Callable

from subsystem.captainIntake import CaptainIntake
from constants import CaptainPlanetConsts as intakeConsts

import commands2
from wpilib import Timer


class IntakeManually(commands2.Command):
    def __init__(self, speed: Callable[[], float], intake: CaptainIntake) -> None:
        super().__init__()
        self.speed = speed
        self.intake = intake
        self.addRequirements(intake)

    def execute(self):
        self.intake.setMotor(self.speed(), manualControl=True)

    def end(self, interrupted: bool):
        self.intake.setMotor(0)

    def isFinished(self):
        return False


class IntakeToFront(commands2.Command):
    def __init__(
        self, intake: CaptainIntake, holdDurationSeconds: float = 0.0, reverse: bool = False
    ) -> None:
        """
        """
        super().__init__()
        self.intake = intake
        self.holdDurationSeconds = holdDurationSeconds
        self.reverse = reverse
        self.beamBrokenInExecution = False
        self.timer = Timer()
        self.addRequirements(self.intake)

    def execute(self) -> None:
        """
        """
        self.intake.setMotor(intakeConsts.kDefaultSpeed, reverse=self.reverse, manualControl=False)
        if self.intake.frontBeamBroken and (not self.beamBrokenInExecution):
            self.beamBrokenInExecution = True
            self.timer.restart()

    def end(self, interrupted: bool) -> None:
        """
        """
        self.intake.setMotor(0)

    def isFinished(self) -> bool:
        """
        """
        return self.beamBrokenInExecution and (self.timer.get() > self.holdDurationSeconds)


class IntakeToBack(commands2.Command):
    def __init__(
        self, intake: CaptainIntake, holdDurationSeconds: float = 0.0, reverse: bool = False
    ) -> None:
        """
        """
        super().__init__()
        self.intake = intake
        self.holdDurationSeconds = holdDurationSeconds
        self.reverse = reverse
        self.beamBrokenInExecution = False
        self.timer = Timer()
        self.addRequirements(self.intake)

    def execute(self) -> None:
        """
        """
        self.intake.setMotor(intakeConsts.kDefaultSpeed, reverse=self.reverse, manualControl=False)
        if self.intake.backBeamBroken and (not self.beamBrokenInExecution):
            self.beamBrokenInExecution = True
            self.timer.restart()

    def end(self, interrupted: bool) -> None:
        """
        """
        self.intake.setMotor(0)

    def isFinished(self) -> bool:
        """
        """
        return self.beamBrokenInExecution and (self.timer.get() > self.holdDurationSeconds)


class IntakeToFrontOnly(commands2.Command):
    def __init__(self, intake: CaptainIntake) -> None:
        """
        """
        super().__init__()
        self.intake = intake
        self.addRequirements(self.intake)

    def execute(self) -> None:
        """
        """
        self.intake.setMotor(intakeConsts.kDefaultSpeed, reverse=False, manualControl=False)

    def end(self, interrupted: bool) -> None:
        """
        """
        self.intake.setMotor(0)

    def isFinished(self) -> bool:
        """
        """
        return self.intake.frontBeamBroken and (not self.intake.backBeamBroken)


class IntakeReleasePiece(commands2.Command):
    def __init__(self, intake: CaptainIntake, postBeamDuration: float = 0.0) -> None:
        """
        """
        super().__init__()
        self.intake = intake
        self.postBeamDuration = postBeamDuration
        self.beamOpenedInExecution = False
        self.timer = Timer()
        self.addRequirements(self.intake)

    def execute(self) -> None:
        """
        """
        self.intake.setMotor(intakeConsts.kDefaultSpeed, reverse=False, manualControl=False)
        if (not self.intake.frontBeamBroken) and (not self.beamOpenedInExecution):
            self.beamOpenedInExecution = True
            self.timer.restart()

    def end(self, interrupted: bool) -> None:
        """
        """
        self.intake.setMotor(0)

    def isFinished(self) -> bool:
        """
        """
        return self.beamOpenedInExecution and (self.timer.get() > self.postBeamDuration)


def determineHoldAction(intake: CaptainIntake) -> str:
    if intake.frontBeamBroken and intake.backBeamBroken:
        return intakeConsts.BreakBeamActionOptions.DONOTHING
    if intake.frontBeamBroken:
        return intakeConsts.BreakBeamActionOptions.TOBACK
    if intake.backBeamBroken:
        return intakeConsts.BreakBeamActionOptions.TOFRONT
    return intakeConsts.BreakBeamActionOptions.DONOTHING


def generateIntakeMaintainHold(intake: CaptainIntake) -> commands2.Command:
    return commands2.cmd.select(
        {
            intakeConsts.BreakBeamActionOptions.DONOTHING: commands2.cmd.run(lambda: intake.setMotor(0), intake),
            intakeConsts.BreakBeamActionOptions.TOBACK: IntakeToBack(intake, holdDurationSeconds=0.0, reverse=True),
            intakeConsts.BreakBeamActionOptions.TOFRONT: IntakeToFront(intake, holdDurationSeconds=0.0, reverse=False)
        },
        determineHoldAction
    )


def generateIntakeFrontForwardHold(intake: CaptainIntake) -> commands2.Command:
    return commands2.cmd.sequence(
        IntakeToFrontOnly(intake),
        IntakeToBack(intake, holdDurationSeconds=0.1, reverse=True)
    )
