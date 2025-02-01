"""
Collection of numeric constants that define physical properties of the robot
"""

# Native imports
import math

#############################
# ROBOT ###################
#############################


class RobotConstants:
    massKG: float = 74.088
    MOI: float = 6.883


#############################
# SENSORS ###################
#############################


class SparkMaxConstants:
    faultRateMs: int = 50
    motorPosRateMs: int = 20
    appliedOutputRateMs: int = 10
    motorTelmRateMs: int = 50
    analogRateMs: int = 1833
    altEncoderRateMs: int = 1050
    dutyCycleEncRateMs: int = 2150
    dutyCycleEncVelRateMs: int = 3150


#############################
# SWERVE ###################
#############################


#* Coordinates
#* Wheel dimensions
#? Swerve level & gear ratios
#* Absolute encoder offsets
#* Gyro CCW pos - invert
#* Steer and drive - CCW pos - invert
# Printout to true direction verification
# Teleop control
# Verify Odometer
# PID control config
# Auto control
# Velocity verification


# Y: 22 inches wide, end-to-end
# Y: +/- 0.26035
# X: 26 inches long, end-to-end
# X: +/- 0.31115
# Wheel width: 1.5 inches
# Wheel diameter: 4 inches
# Wheel COF (Colson): 1

# Swerve Level: L2?
# Swerve Drive Gear Ratio: 
# Swerve Steer Gear Ratio: 

# frontLeft raw abs: 0.329590
# frontRight raw abs: 0.988525 ...281
# backLeft raw abs: 0.996826
# backRight raw abs: 0.834961

# Gryo - do not invert

# Front Left, Drive: invert
# Front Left, Steer: invert
# Front Right, Drive: invert
# Front Right, Steer: invert
# Back Left, Drive: invert
# Back Left, Steer: invert
# Back Right, Drive: invert
# Back Left, Steer: invert

class SwerveDriveConsts(RobotConstants):
    moduleFrontLeftX: float = 0.31115
    moduleFrontLeftY: float = 0.26035
    moduleFrontRightX: float = 0.31115
    moduleFrontRightY: float = -0.26035
    moduleBackLeftX: float = -0.31115
    moduleBackLeftY: float = 0.26035
    moduleBackRightX: float = -0.31115
    moduleBackRightY: float = -0.26035

    invertGyro: bool = False
    moduleFrontLeftInvertDrive: bool = True
    moduleFrontRightInvertDrive: bool = True
    moduleBackLeftInvertDrive: bool = True
    moduleBackRightInvertDrive: bool = True

    moduleFrontLeftInvertSteer: bool = True
    moduleFrontRightInvertSteer: bool = True
    moduleBackLeftInvertSteer: bool = True
    moduleBackRightInvertSteer: bool = True

    maxTranslationMPS: float = 4.14528
    maxAngularDPS: float = math.degrees(maxTranslationMPS / math.hypot(moduleFrontLeftY, moduleFrontLeftX))


class SwerveModuleMk4iConsts(SwerveDriveConsts):
    """
    https://github.com/SwerveDriveSpecialties/swerve-lib/blob/develop/src/main/java/com/swervedrivespecialties/swervelib/ctre/Falcon500DriveControllerFactoryBuilder.java
    """
    kNominalVoltage: float = 12.0
    kDriveCurrentLimit: int = 40
    kSteerCurrentLimit: int = 20
    kRampRate: float = 0.25
    kTicksPerRotation: int = 1
    kCanStatusFrameHz: int = 10
    quadratureMeasurementRateMs: int = 10
    quadratureAverageDepth: int = 2
    numDriveMotors: int = 1
    motorType: str = "NEO" # should be an option in wpimath.system.plant.DCMotor


class SwerveModuleMk4iL1Consts(SwerveModuleMk4iConsts):
    """
    https://docs.yagsl.com/configuring-yagsl/standard-conversion-factors
    """
    wheelDiameter: float = 0.10033
    driveGearRatio: float = 8.14
    steerGearRatio: float = 150 / 7
    
    drivePositionConversionFactor : float = (math.pi * wheelDiameter) / (driveGearRatio * SwerveModuleMk4iConsts.kTicksPerRotation)
    driveVelocityConversionFactor: float = drivePositionConversionFactor / 60.0
    steerPositionConversionFactor: float = 360 / (steerGearRatio * SwerveModuleMk4iConsts.kTicksPerRotation)
    steerVelocityConversionFactor: float = steerPositionConversionFactor / 60.0

    moduleType: str = "Mk4i_L1"


class SwerveModuleMk4iL2Consts(SwerveModuleMk4iConsts):
    """
    https://docs.yagsl.com/configuring-yagsl/standard-conversion-factors
    """
    wheelDiameter: float = 0.10033
    wheelCOF: float = 1.0
    driveGearRatio: float = 6.75
    steerGearRatio: float = 150 / 7

    drivePositionConversionFactor : float = (math.pi * wheelDiameter) / (driveGearRatio * SwerveModuleMk4iConsts.kTicksPerRotation)
    driveVelocityConversionFactor: float = drivePositionConversionFactor / 60.0
    steerPositionConversionFactor: float = 360 / (steerGearRatio * SwerveModuleMk4iConsts.kTicksPerRotation)
    steerVelocityConversionFactor: float = steerPositionConversionFactor / 60.0

    moduleType: str = "Mk4i_L2"
