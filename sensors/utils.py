# Internal imports
from constants import SparkMaxConstants

# Third-party imports
import rev


def configureSparkMaxCanRates(
    config: rev.SparkMaxConfig,
    drive_motor_flag: bool,
    faultRateMs: int = SparkMaxConstants.faultRateMs,
    motorPosRateMs: int = SparkMaxConstants.motorPosRateMs,
    appliedOutputRateMs: int = SparkMaxConstants.appliedOutputRateMs
) -> None:
    """
    Configure the data transfer type and rate for swerve drive SparkMaxs. Some configurations
    are universal while others vary based on the given inputs.

    Args:
        config: the configuration object for the SparkMax to update
        drive_motor_flag: if True, the SparkMax controls a drive motor on the swerve drive.
            If False, it controls a steer motor
        faultRateMs: the rate, in milliseconds, at which fault signals are transmitted
        motorPosRateMs: the rate, in milliseconds, at which the position of the motor is transmitted
        appliedOutputRateMs: the rate, in milliseconds, at which the motor's applied output is transmitted

    Returns:
        None - passed configuration is updated in-place
    """
    (
        config.signals
        # Fixed settings
        .absoluteEncoderPositionAlwaysOn(False)
        .absoluteEncoderVelocityAlwaysOn(False)
        .analogPositionAlwaysOn(False)
        .analogVelocityAlwaysOn(False)
        .analogVoltageAlwaysOn(False)
        .externalOrAltEncoderPositionAlwaysOn(False)
        .externalOrAltEncoderVelocityAlwaysOn(False)
        .primaryEncoderPositionAlwaysOn(True)
        .IAccumulationAlwaysOn(False)

        # Input settings
        .primaryEncoderVelocityAlwaysOn(drive_motor_flag)
        .primaryEncoderPositionPeriodMs(motorPosRateMs)
        .appliedOutputPeriodMs(appliedOutputRateMs)
        .faultsPeriodMs(faultRateMs)
    )
