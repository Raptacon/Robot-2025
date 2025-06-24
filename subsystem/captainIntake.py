import wpilib
import rev
import commands2

from constants import CaptainPlanetConsts as intakeConsts, DiverCarlChuteConsts as chuteConsts


class CaptainIntake(commands2.Subsystem):
    """
    """
    def __init__(self) -> None:
        super().__init__()
        self.intakeMotor = rev.SparkFlex(
            intakeConsts.kMotorCanId, rev.SparkLowLevel.MotorType.kBrushless
        )
        self.chuteMotor = rev.SparkFlex(
            chuteConsts.kMotorCanId, rev.SparkLowLevel.MotorType.kBrushless
        )
        # Front is closest to pivot motor
        self.frontBreakbeam = wpilib.DigitalInput(intakeConsts.kFrontBreakBeam)
        self.backBreakbeam = wpilib.DigitalInput(intakeConsts.kBackBreakBeam)

        self.configureIntakeMotor()
        self.configureChuteMotor()

        self.updateSensorRecordings()

    def configureIntakeMotor(self) -> None:
        """
        """
        motor_config = rev.SparkBaseConfig()

        (
            motor_config
            .setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
            .inverted(intakeConsts.kMotorInverted)
            .smartCurrentLimit(intakeConsts.kCurrentLimitAmps)
        )

        # Apply the configuration and burn to the SparkFlex's flash memory
        self.intakeMotor.configure(
            motor_config, rev.SparkBase.ResetMode.kNoResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters
        )

    def configureChuteMotor(self) -> None:
        """
        """
        motor_config = rev.SparkBaseConfig()

        (
            motor_config
            .setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
            .inverted(chuteConsts.kMotorInverted)
            .smartCurrentLimit(chuteConsts.kCurrentLimitAmps)
        )

        # Apply the configuration and burn to the SparkFlex's flash memory
        self.chuteMotor.configure(
            motor_config, rev.SparkBase.ResetMode.kNoResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters
        )

    def setMotor(self, speed: float, reverse: bool = False, manualControl: bool = False) -> None:
        """
        """
        speedUseIntake = speed
        speedUseChute = speed
        if reverse:
            speedUseIntake = -1 * speedUseIntake
            speedUseChute = -1 * speedUseChute
        if manualControl:
            speedUseIntake = speedUseIntake * intakeConsts.kOperatorDampener
            speedUseChute = speedUseChute * chuteConsts.kOperatorDampener
        self.intakeMotor.set(speedUseIntake)
        self.chuteMotor.set(speedUseChute)

    def updateSensorRecordings(self) -> None:
        """
        """
        self.frontBeamBroken = not self.frontBreakbeam.get()
        self.backBeamBroken = not self.backBreakbeam.get()
        wpilib.SmartDashboard.putBoolean("Front broken", self.frontBeamBroken)
        wpilib.SmartDashboard.putBoolean("Back broken", self.backBeamBroken)

    def periodic(self) -> None:
        """
        """
        self.updateSensorRecordings()
