import commands2
import rev

class SparkyIntake(commands2.SubsystemBase):
    def __init__(self) -> None:
        self.intakeMotor = rev.SparkMax(21, rev.SparkLowLevel.MotorType.kBrushless)

    def runIntake(self, percent : float):
        self.intakeMotor.set(percent)
