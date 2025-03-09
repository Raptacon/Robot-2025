# Native imports
import math

# Internal imports
from constants import DiverCarlElevatorConsts as c

# Third-party imports
import commands2
import rev
from wpilib import Timer, SmartDashboard
from wpimath.trajectory import TrapezoidProfile
from wpimath.controller import ElevatorFeedforward

class DiverCarlElevator(commands2.Subsystem):
    def __init__(self, elevatorHeightDelta: float = c.kMechDeltaHeightCm, dt=0.02) -> None:
        super().__init__()
        self._dt = dt
        self._heightDelta = elevatorHeightDelta
        self._primaryMotor = rev.SparkMax(c.kMotorPrimaryCanId, rev.SparkLowLevel.MotorType.kBrushless)

        # setup primary
        motorConfig = rev.SparkBaseConfig()
        motorConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        motorConfig.inverted(c.kMotorPrimaryInverted)
        motorConfig.smartCurrentLimit(c.kCurrentLimit)

        #enable soft forward limits
        softLimit = rev.SoftLimitConfig()
        softLimit.forwardSoftLimit(c.kSoftLimits["forwardLimit"])
        softLimit.forwardSoftLimitEnabled(c.kSoftLimits["forward"])
        softLimit.reverseSoftLimit(c.kSoftLimits["reverseLimit"])
        softLimit.reverseSoftLimitEnabled(c.kSoftLimits["reverse"])

        motorConfig.apply(softLimit)

        #enable hard limits
        hardLimits = rev.LimitSwitchConfig()
        hardLimits.forwardLimitSwitchEnabled(c.kLimits["forward"])
        hardLimits.forwardLimitSwitchType(c.kLimits["forwardType"])
        hardLimits.reverseLimitSwitchEnabled(c.kLimits["reverse"])
        hardLimits.reverseLimitSwitchType(c.kLimits["reverseType"])
        motorConfig.apply(hardLimits)

        # Setup encoder
        self._encoder = self._primaryMotor.getEncoder()
        encConfig = rev.EncoderConfig()
        #should be set to rot and set/get heights convert to engineering units based on physical design
        encConfig.positionConversionFactor(1.0)
        encConfig.velocityConversionFactor(1.0)

        motorConfig.apply(encConfig)

        self._trap_profile = TrapezoidProfile(TrapezoidProfile.Constraints(3, 1.5))

        #setup PID Slot 0 - normal
        motorConfig.closedLoop.FeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        motorConfig.closedLoop.pidf(*c.kPidf0)
        motorConfig.closedLoop.outputRange(*c.kMaxOutRange0)

        self._primaryMotor.configure(motorConfig, rev.SparkBase.ResetMode.kNoResetSafeParameters , rev.SparkBase.PersistMode.kPersistParameters)

        self._controller = self._primaryMotor.getClosedLoopController()
        self._controller.setReference(0, self._primaryMotor.ControlType.kPosition)

        self._disabled = False
        self._curentGoal = 0.0

        self._setpoint = self.resetSetpoint()
        self.known_positions = {
            "L1": c.kL1,
            "L2": c.kL2,
            "L3": c.kL3,
            "L4": c.kL4,
        }

    def periodic(self) -> None:
        if self.getReverseLimit():
            self.resetSetpoint()
            self._encoder.setPosition(0)

        if self.getForwardLimit() and self._primaryMotor.get() > 0:
            self._primaryMotor.set(0)
            return

        if self.getReverseLimit() and self._primaryMotor.get() < 0:
            self._primaryMotor.set(0)
            return

        if self._disabled:
           self._primaryMotor.disable()
           return

        # Setup profiler
        goal_state = TrapezoidProfile.State(self._curentGoal, 0.0)
        self._setpoint = self._trap_profile.calculate(self._dt, self._setpoint, goal_state)

        SmartDashboard.putNumber("Profile elevator velocity", self._setpoint.velocity)
        SmartDashboard.putNumber("Profile elevator position", self._setpoint.position)

        self._controller.setReference(
            self._curentGoal, self._primaryMotor.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0
        )

    def getRotFromCm(self, cm: float):
        return cm * c.kEncoderFullRangeRot / c.kFullRangeHeighCm

    def getCmFromRot(self, rot: float):
        return rot * c.kFullRangeHeighCm / c.kEncoderFullRangeRot

    def getForwardLimit(self) -> bool:
        """Returns if either motor controller has hit the forward limit switch
        Returns:
            bool: True if either motor controller has hit the forward limit switch
        """
        return self._primaryMotor.getForwardLimitSwitch().get()

    def getReverseLimit(self) -> bool:
        return self._primaryMotor.getReverseLimitSwitch().get()

    def resetSetpoint(self) -> TrapezoidProfile.State:
        return TrapezoidProfile.State(0, 0)

    def setHeight(self, heightCm: float) -> None:
        """
        Sets the height of the elevator goal in meters from ground.
        """
        if self._disabled:
            self._disabled = False
            #self._controller.setReference(self._encoder.getPosition())

        heightCm = heightCm - self._heightDelta

        if heightCm < 0:
            heightCm = 0

        # cache goal for easy access and to use in periodic
        self._curentGoal = self.getRotFromCm(heightCm)

    def getSetHeightCm(self) -> float:
        """
        Returns the current goal height of the elevator in meters from ground.
        Returns:
            float: set heigh in meters
        """
        return self.getCmFromRot(self._curentGoal) + self._heightDelta

    def getHeightCm(self) -> float:
        """Returns the current height of the elevator in meters from ground.
        """
        return self.getCmFromRot(self._encoder.getPosition()) + self._heightDelta

    def atGoal(self) -> bool:
        """
        Returns if the elevator is at the goal height.
        """
        return math.isclose(self._encoder.getVelocity(), 0.0, abs_tol=0.01) and math.isclose(self._encoder.getPosition(), self._curentGoal, abs_tol=0.1)

    def getError(self) -> float:
        """
        Returns the error in meters between the goal height and the current height.
        """
        return self.getCmFromRot(self._curentGoal - self._encoder.getPosition())

    def setIncrementalMove(self, deltaM: float) -> None:
        """
        Moves the elevator by the delta in meters from the current goal height.
        """
        self.setHeight(self.getSetHeightCm() + deltaM)

    def stopElevator(self) -> None:
        """
        Stops the by setting current height to goal. This will maintain the current height.
        """
        self.setHeight(self.getHeightCm())

    def disable(self) -> None:
        """
        Disables the elvators motors. The elevator may fall under gravity.
        """
        self._disabled = True

    def getDisabled(self) -> bool:
        return self._disabled
