import wpilib
import math
import commands2
import rev
from constants import DiverCarlElevatorConsts as c

# plan to use 25:1 gear ratio
# drum TBD
# 2 motors one will run opposite direction


class DiverCarlElevator(commands2.Subsystem):

    def __init__(
        self, elevatorHeightDelta: float = c.kMechDeltaHeightCm, dt=0.02
    ) -> None:
        super().__init__()
        self._dt = dt
        self._heightDelta = elevatorHeightDelta
        self._primaryMotor = rev.SparkMax(
            c.kMotorPrimaryCanId, rev.SparkLowLevel.MotorType.kBrushless
        )
        # setup primary
        motorConfig = rev.SparkBaseConfig()
        motorConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        motorConfig.inverted(c.kMotorPrimaryInverted)

        # enable soft forward limits
        softLimit = rev.SoftLimitConfig()
        softLimit.forwardSoftLimit(c.kSoftLimits["forwardLimit"])
        softLimit.forwardSoftLimitEnabled(c.kSoftLimits["forward"])
        softLimit.reverseSoftLimit(c.kSoftLimits["reverseLimit"])
        softLimit.reverseSoftLimitEnabled(c.kSoftLimits["reverse"])

        motorConfig.apply(softLimit)

        # enable hard limits
        hardLimits = rev.LimitSwitchConfig()
        hardLimits.forwardLimitSwitchEnabled(c.kLimits["forward"])
        hardLimits.forwardLimitSwitchType(c.kLimits["forwardType"])
        hardLimits.reverseLimitSwitchEnabled(c.kLimits["reverse"])
        hardLimits.reverseLimitSwitchType(c.kLimits["reverseType"])
        motorConfig.apply(hardLimits)

        self._encoder = self._primaryMotor.getEncoder()
        encConfig = rev.EncoderConfig()
        # should be set to rot and set/get heights convert to engineering units based on physical design
        encConfig.positionConversionFactor(1.0)
        encConfig.velocityConversionFactor(1.0)

        motorConfig.apply(encConfig)

        # setup PID Slot 0 - normal
        motorConfig.closedLoop.FeedbackSensor(
            rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        )
        motorConfig.closedLoop.pidf(*c.kPidf0)
        motorConfig.closedLoop.outputRange(*c.kMaxOutRange0)

        # TODO Setup PID Slot 1 - slow?

        self._primaryMotor.configure(
            motorConfig,
            rev.SparkBase.ResetMode.kNoResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters,
        )

        self._controller = self._primaryMotor.getClosedLoopController()
        self._controller.setReference(0, self._primaryMotor.ControlType.kPosition)
        # TODO add profile states

        self._disabled = False
        self._curentGoal = 0.0

        # setup telem
        self._limitAlert = wpilib.Alert(
            "mechanism", "Elevator Limit Reached", wpilib.Alert.AlertType.kWarning
        )
        self._limitAlert.set(False)
        self._trackingAlert = wpilib.Alert(
            "mechanism", "Elevator moving", wpilib.Alert.AlertType.kInfo
        )
        self._trackingAlert.set(False)

    def initSendable(self, builder):
        super().initSendable(builder)
        builder.addFloatProperty("CurrentCm", self.getHeightCm)
        builder.addBooleanProperty("AtGoal", self.atGoal, None)
        builder.addFloatProperty("GoalCm", self.getSetHeightCm, self.setHeight)
        builder.addFloatProperty("ErrorCm", self.getError)
        builder.addBooleanProperty("ForwardLimit", self.getForwardLimit, None)
        builder.addBooleanProperty("ReverseLimit", self.getReverseLimit, None)
        builder.addBooleanProperty("Disabled", self.getDisabled, self.disable)
        exit(-1)

    def periodic(self) -> None:
        # update telemtry
        if self.atGoal():
            self._trackingAlert.setText(
                f"Elevator stable at {self._encoder.getPosition():1.1f}cm"
            )
            self._trackingAlert.set(True)
        else:
            self._trackingAlert.setText(
                f"Elevator at {self._encoder.getPosition():1.1f}cm goal {self.getSetHeightCm():1.1f}cm"
            )
            self._trackingAlert.set(True)

        if self.getReverseLimit():
            self._encoder.setPosition(0)

        # safe the motors if forward limit is hit
        if self.getForwardLimit():
            self._limitAlert.setText(
                f"Elevator TOP HIT at {self._encoder.getPosition()}cm"
            )
            self._limitAlert.set(True)
            return

        if self.getReverseLimit() and self._curentGoal > 0:
            self._primaryMotor.set(0)
            self._limitAlert.setText(
                f"Elevator BOTTOM HIT at {self._encoder.getPosition()}cm"
            )
            self._limitAlert.set(True)
            return

        # clear the alert as we no longer are at the limit
        self._limitAlert.set(False)

        if self._disabled:
            self._primaryMotor.disable()
            return

        self._controller.setReference(
            self._curentGoal, self._primaryMotor.ControlType.kPosition
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
        return (
            self._primaryMotor.getForwardLimitSwitch().get()
            or self._followerMotor.getForwardLimitSwitch().get()
        )

    def getReverseLimit(self) -> bool:
        return (
            self._primaryMotor.getReverseLimitSwitch().get()
            or self._followerMotor.getReverseLimitSwitch().get()
        )

    def getForwardLimit(self) -> bool:
        """Returns if either motor controller has hit the forward limit switch
        Returns:
            bool: True if either motor controller has hit the forward limit switch
        """
        # TODO add soft limits?
        return self._primaryMotor.getForwardLimitSwitch().get()

    def getReverseLimit(self) -> bool:
        # TODO add soft limits?
        return self._primaryMotor.getReverseLimitSwitch().get()

    def setHeight(self, heightCm: float) -> None:
        """
        Sets the height of the elevator goal in meters from ground.
        """
        # TODO add max and min set constraints to setters
        if self._disabled:
            self._disabled = False
            self._controller.reset(self._encoder.getDistance())

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
        """Returns the current height of the elevator in meters from ground."""
        return self.getCmFromRot(self._encoder.getPosition()) + self._heightDelta

    def atGoal(self) -> bool:
        """
        Returns if the elevator is at the goal height.
        """
        return math.isclose(
            self._encoder.getVelocity(), 0.0, abs_tol=0.01
        ) and math.isclose(self._encoder.getPosition(), self._curentGoal, abs_tol=0.1)

    def getError(self) -> float:
        """
        Returns the error in meters between the goal height and the current height.
        """
        return self.getCmFromRot(self._curentGoal - self._encoder.getPosition())

    def setIncrementalMove(self, deltaM: float) -> None:
        """
        Moves the elevator by the delta in meters from the current goal height.
        """
        self.setHeight(self.getSetHeightM() + deltaM)

    def stopElevator(self) -> None:
        """
        Stops the by setting current height to goal. This will maintain the current height.
        """
        self.setHeight(self.getHeightM())

    def disable(self) -> None:
        """
        Disables the elvators motors. The elevator may fall under gravity.
        """
        self._disabled = True

    def getDisabled(self) -> bool:
        return self._disabled
