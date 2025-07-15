"""
Collection of numeric constants that define physical properties of the robot
"""

# Native imports
import math
from enum import Enum

# Third-Party Imports
import rev
from raptacon3200.constants.swerve_constants import SwerveDriveConsts, SwerveModuleMk4iConsts


#############################
# ROBOT ###################
#############################


class RobotConstants:
    massKG: float = 63.9565
    #MOI: Moment of inertia, kg*m^2
    MOI: float = 5.94175290870316

#############################
# SWERVE ###################
#############################

swerve_drive_constants = SwerveDriveConsts(
    massKG=RobotConstants.massKG,
    MOI=RobotConstants.MOI,
    moduleFrontLeftX=0.31115,
    moduleFrontLeftY=0.26035,
    moduleFrontRightX=0.31115,
    moduleFrontRightY=-0.26035,
    moduleBackLeftX=-0.31115,
    moduleBackLeftY=0.26035,
    moduleBackRightX=-0.31115,
    moduleBackRightY=-0.26035,
    invertGyro=False,
    moduleFrontLeftInvertDrive=True,
    moduleFrontRightInvertDrive=True,
    moduleBackLeftInvertDrive=True,
    moduleBackRightInvertDrive=True,
    moduleFrontLeftInvertSteer=True,
    moduleFrontRightInvertSteer=True,
    moduleBackLeftInvertSteer=True,
    moduleBackRightInvertSteer=True,
    maxTranslationMPS=4.14528,
)

swerve_module_constants = SwerveModuleMk4iConsts(
    kNominalVoltage=12.0,
    kDriveCurrentLimit=40,
    kSteerCurrentLimit=20,
    kRampRate=0.25,
    kTicksPerRotation=1,
    kCanStatusFrameHz=10,
    quadratureMeasurementRateMs=10,
    quadratureAverageDepth=2,
    numDriveMotors=1,
    motorType="NEO",
    wheelDiameter=0.10033,
    driveGearRatio=8.14,
    steerGearRatio=150 / 7,
    moduleType="Mk4i_L2",
)

#############################
# OTHER ###################
#############################

class DiverCarlChisteraConsts():
    kMotorPrimaryCanId = 12
    kMotorPrimaryInverted = False
    kEncoderFullRangeRot = 19.0 #soft limit
    kFullRangeDegrees = 180 #Estimate
    kSoftLimits = {"forward": True, "forwardLimit": 0, "reverse": True, "reverseLimit": -1.0}
    kLimits = {"forward": True,
                "forwardType": rev.LimitSwitchConfig.Type.kNormallyOpen,
                "reverse": False,
                "reverseType": rev.LimitSwitchConfig.Type.kNormallyOpen}

    kPidf0 = (0.3, 0.001 , 0, 0, rev.ClosedLoopSlot.kSlot0) # P I D F Slot
    kMaxOutRange0 = (-0.25, 0.7, rev.ClosedLoopSlot.kSlot0) # Min Max Slot


class DiverCarlElevatorConsts:
    kCurrentLimitAmps = 40
    kMotorCanId = 11
    kMaxHeightAboveZeroCm = 180
    kRotationsToMaxHeight = 101
    kHeightAtZeroCm = 10.16
    kMotorInverted = False
    kTrapezoidProfileUp = (135 * 1.25, 150 * 1.25)  # Max Vel (cm/s) Max Accel (cm/s^2)
    kTrapezoidProfileDown = (75, 37.5)
    kFeedforward = (0, 0.28, 0.1, 0)  # kS kG kV kA
    kPid0 = (0.05, 0, 0, rev.ClosedLoopSlot.kSlot0)  # P I D Slot
    kMaxOutRange0 = (-0.35, 1.0, rev.ClosedLoopSlot.kSlot0)  # Min Max Slot
    kSoftLimits = {
        "forward": True,
        "forwardLimit": 178,
        "reverse": False,
        "reverseLimit": 0,
    }
    kLimits = {
        "forward": True,
        "forwardType": rev.LimitSwitchConfig.Type.kNormallyOpen,
        "reverse": True,
        "reverseType": rev.LimitSwitchConfig.Type.kNormallyOpen,
    }
    # Offsets from the top of the reef pole and the elevator position (from the ground) for the chute,
    # in centimeters
    kL1OffsetCm = 0
    kL2OffsetCm = 0
    kL3OffsetCm = 0
    kL4OffsetCm = 0
    kChuteHeightCm = 10.16


class DiverCarlChuteConsts:
    kMotorCanId = 14
    kMotorInverted = True
    kCurrentLimitAmps = 50
    kDefaultSpeed = 0.5
    kOperatorDampener = 0.15


class CaptainPlanetConsts:
    kMotorCanId = 13
    kMotorInverted = False
    kCurrentLimitAmps = 30
    kFrontBreakBeam = 0
    kBackBreakBeam = 1
    kDefaultSpeed = 0.15
    kOperatorDampener = 0.15
    class BreakBeamActionOptions(Enum):
        DONOTHING = 1
        TOFRONT = 2
        TOBACK = 3


class MechConsts:
    kArmRestPosition = 0.0 # movement arc
    kArmRestPosTol = 0.02 # % of movement arc
    kArmSafPosTol = 0.02 # % of movement arc
    kArmSafeAngleStart = 0.085 # movement arc
    kArmSafeAngleEnd = 0.13 # movement arc
    kArmVertical = 0.259
    kArmLevel2Position = 0.153
    kArmAngleIncrement = 0.1
    kElevatorSafeHeight = 5 #cm
    kElevatorTrough = 46 #cm
    kArmAngleTrough = 0.11
    kElevatorReef2 = 68 #cm # THIS IS A GUESS
    kArmAngleReef2 = 0.11
    kElevatorReef3 = 115 #cm # THIS IS A GUESS
    kArmAngleReef3 = 0.11
    kElevatorReef4 = 183 #cm
    kArmAngleReef4 = 0.15

class PoseOptions(Enum):
    MANUAL = -1
    REST = 0
    TROUGH = 1
    REEF2 = 2
    REEF3 = 3
    REEF4 = 4
    ALGAE2 = 5
    ALGAE3 = 6
