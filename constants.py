"""
Collection of numeric constants that define physical properties of the robot
"""

# Native imports
import math

# Third-Party Imports
import rev

#############################
# ROBOT ###################
#############################


class RobotConstants:
    massKG: float = 74.088
    #MOI: Moment of inertia, kg*m^2
    MOI: float = 6.883


#############################
# SWERVE ###################
#############################


class SwerveDriveConsts(RobotConstants):
    # where the wheel is compared to the center of the robot in meters
    moduleFrontLeftX: float = 0.31115
    moduleFrontLeftY: float = 0.26035
    moduleFrontRightX: float = 0.31115
    moduleFrontRightY: float = -0.26035
    moduleBackLeftX: float = -0.31115
    moduleBackLeftY: float = 0.26035
    moduleBackRightX: float = -0.31115
    moduleBackRightY: float = -0.26035

    # inverts if the module or gyro does not rotate counterclockwise positive
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
    # cuurrent limits use amps
    kDriveCurrentLimit: int = 40
    kSteerCurrentLimit: int = 20
    # ramp rate: how fast the bot can go from 0% to 100%, mesured in seconds
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
    wheelDiameter: float = 0.10033 # in meters
    driveGearRatio: float = 8.14
    steerGearRatio: float = 150 / 7

    # position: meters per rotation
    # velocity: meters per second
    drivePositionConversionFactor : float = (math.pi * wheelDiameter) / (driveGearRatio * SwerveModuleMk4iConsts.kTicksPerRotation)
    driveVelocityConversionFactor: float = drivePositionConversionFactor / 60.0
    steerPositionConversionFactor: float = 360 / (steerGearRatio * SwerveModuleMk4iConsts.kTicksPerRotation)
    steerVelocityConversionFactor: float = steerPositionConversionFactor / 60.0

    moduleType: str = "Mk4i_L1"


class SwerveModuleMk4iL2Consts(SwerveModuleMk4iConsts):
    """
    https://docs.yagsl.com/configuring-yagsl/standard-conversion-factors
    """
    wheelDiameter: float = 0.10033 # in meters
    # COf: coefficient, force/force (no units)
    wheelCOF: float = 1.0
    driveGearRatio: float = 6.75
    steerGearRatio: float = 150 / 7

    # position: meters per rotation
    # velocity: meters per second
    drivePositionConversionFactor : float = (math.pi * wheelDiameter) / (driveGearRatio * SwerveModuleMk4iConsts.kTicksPerRotation)
    driveVelocityConversionFactor: float = drivePositionConversionFactor / 60.0
    steerPositionConversionFactor: float = 360 / (steerGearRatio * SwerveModuleMk4iConsts.kTicksPerRotation)
    steerVelocityConversionFactor: float = steerPositionConversionFactor / 60.0

    moduleType: str = "Mk4i_L2"


class DiverCarlElevatorConsts():
    kCurrentLimitAmps = 40
    kMotorCanId = 11
    kMaxHeightAboveZeroCm = 180
    kRotationsToMaxHeight = 101
    kHeightAtZeroCm = 10.16
    kMotorInverted = False
    kTrapezoidProfileUp = (135, 150) # Max Vel (cm/s) Max Accel (cm/s^2)
    kTrapezoidProfileDown = (35, 37.5)
    kFeedforward = (0, 0.28, 0.1, 0) # kS kG kV kA
    kPid0 = (0.05, 0, 0, rev.ClosedLoopSlot.kSlot0) # P I D Slot
    kMaxOutRange0 = (-1, 1, rev.ClosedLoopSlot.kSlot0) # Min Max Slot
    kSoftLimits = {"forward": True, "forwardLimit": 178, "reverse": False, "reverseLimit": 0}
    kLimits = {
        "forward": True,
        "forwardType": rev.LimitSwitchConfig.Type.kNormallyOpen,
        "reverse": True,
        "reverseType": rev.LimitSwitchConfig.Type.kNormallyOpen
    }
    # Offsets from the top of the reef pole and the elevator position (from the ground) for the chute,
    # in centimeters
    kL1OffsetCm = 0
    kL2OffsetCm = 0
    kL3OffsetCm = 0
    kL4OffsetCm = 0
    kChuteHeightCm = 10.16


class DiverCarlChisteraConsts():
    kMotorCanId = 12
    kMotorInverted = False
    kEncoderTicksToDegrees = 360
    kEncoderZeroOffsetDegres = 0
    kMinDeg = 0
    kMaxDeg = 250
    kMaxVelRPS = 6
    kMaxAccelRPSS = 0.5
    kPid = (1.3, 0, 0.7)


class DiverCarlChute():
    kMotorCanId = 14
    kBreakBeamPort = 3
    kMotorInverted = False
    kCurrentLimitAmps = 20
    kDefaultSpeed = 0.3

class CaptainPlanetConsts():
    kMotorCanId = 13
    kMotorInverted = False
    kFrontBreakBeam = 2
    kBackBreakBeam = 3
    kDefaultSpeed = -0.25
