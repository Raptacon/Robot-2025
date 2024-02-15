import commands2
import rev
import wpilib

class SwerveShooter(commands2.SubsystemBase):
    def __init__(self) -> None:
        self.intakeMotor = rev.CANSparkMax(23, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.intakeMotor.setInverted(True)
        self.leftShootMotor = rev.CANSparkMax(24, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.rightShooterMotor = rev.CANSparkMax(25, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.rightShooterMotor.setInverted(True)

    def runIntake(self, speed : float):
        self.intakeMotor.set(speed)

    def runShooters(self, speed : float):
        self.leftShootMotor.set(speed)
        self.rightShooterMotor.set(speed)
