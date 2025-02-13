import commands2
import typing
from subsystem.sparkyIntake import SparkyIntake
from subsystem.sparkyIntakePivotController import pivotController

class Intake(commands2.Command):
    def __init__(
        self,
        intake: SparkyIntake,
        pivot : pivotController,
        intakePercent: typing.Callable[[], float],
        toGround: typing.Callable[[], bool]
    ) -> None:
        super().__init__()
        self.intake = intake
        self.pivot = pivot
        self.intakePercent = intakePercent
        self.toGround = toGround

        self.addRequirements(self.intake, self.pivot)

    def execute(self):
        self.intake.runIntake(self.intakePercent())
            
        if(self.toGround()):
            self.pivot.setGroundPickup()
        else:
            self.pivot.setSpitShot()

    def end(self, interrupted):
        self.intake.runIntake(0)
        self.pivot.setHandOffPickup()
