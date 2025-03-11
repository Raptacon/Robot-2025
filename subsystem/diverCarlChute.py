# Internal imports
from constants import DiverCarlChute as c

# Third-party imports
import commands2
import rev
import wpilib


class DiverCarlChute(commands2.Subsystem):
    """
    """
    def __init__(self) -> None:
        """
        """
        super().__init__()
        self.motor = rev.SparkFlex(c.kMotorCanId, rev.SparkLowLevel.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()
        self.encoder.setPosition(0)
        self.breakbeam = wpilib.DigitalInput(c.kBreakBeamPort)

        self.configureMotor()
        self.updateSensorRecordings()

    def configureMotor(self) -> None:
        """
        """
        motor_config = rev.SparkBaseConfig()

        (
            motor_config
            .setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
            .inverted(c.kMotorInverted)
            .smartCurrentLimit(c.kCurrentLimitAmps)
        )

        # Apply the configuration and burn to the SparkFlex's flash memory
        self.motor.configure(
            motor_config, rev.SparkBase.ResetMode.kNoResetSafeParameters, rev.SparkBase.PersistMode.kPersistParameters
        )

    def stopChute(self) -> None:
        """
        """
        self.motor.set(0)

    def runChute(self, reverse: bool = False) -> None:
        """
        """
        speed_use = c.kDefaultSpeed
        if reverse:
            speed_use = -1 * speed_use
        self.motor.set(speed_use)

    def updateSensorRecordings(self) -> None:
        """
        """
        self.motor_speed = self.motor.get()
        self.motor_current = self.motor.getOutputCurrent()
        self.motor_output = self.motor.getAppliedOutput()
        self.motor_velocity = self.encoder.getVelocity()
        self.breakbeam_broken = self.breakbeam.get()

    def periodic(self) -> None:
        """
        """
        self.updateSensorRecordings()
