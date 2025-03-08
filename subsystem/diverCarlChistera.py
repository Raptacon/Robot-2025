import wpilib
import math
import commands2
import rev
from constants import DiverCarlChisteraConsts as c
# 25:1


class DiverCarlChistera(commands2.Subsystem):

    def __init__(self, dt=0.02) -> None:
        super().__init__()
        self._dt = dt
        self._primaryMotor = rev.SparkMax(c.kMotorPrimaryCanId, rev.SparkLowLevel.MotorType.kBrushless)
        # setup primary
        motorConfig = rev.SparkBaseConfig()
        motorConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        motorConfig.inverted(c.kMotorPrimaryInverted)

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

        self._encoder = self._primaryMotor.getEncoder()
        encConfig = rev.EncoderConfig()
        #should be set to 0..-1 control range
        encConfig.positionConversionFactor(1/c.kEncoderFullRangeRot)
        encConfig.velocityConversionFactor(1.0)

        motorConfig.apply(encConfig)

        #setup PID Slot 0 - normal
        motorConfig.closedLoop.FeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        motorConfig.closedLoop.pidf(*c.kPidf0)
        motorConfig.closedLoop.outputRange(*c.kMaxOutRange0)

        #TODO Setup PID Slot 1 - slow?

        self._primaryMotor.configure(motorConfig, rev.SparkBase.ResetMode.kNoResetSafeParameters , rev.SparkBase.PersistMode.kPersistParameters)


        self._controller = self._primaryMotor.getClosedLoopController()
        self._controller.setReference(0, self._primaryMotor.ControlType.kPosition);
        #TODO add profile states

        self._disabled = False
        self._curentGoal = 0.0

        # setup telem
        self._limitAlert = wpilib.Alert("mechanism","Arm Limit Reached", wpilib.Alert.AlertType.kWarning)
        self._limitAlert.set(False)
        self._trackingAlert = wpilib.Alert("mechanism","Arm moving", wpilib.Alert.AlertType.kInfo)
        self._trackingAlert.set(False)


    def periodic(self) -> None:
        # update telemtry
        if self.atGoal():
            self._trackingAlert.setText(f"Arm stable at {self._encoder.getPosition():1.1f}cm")
            self._trackingAlert.set(True)
        else:
            self._trackingAlert.setText(f"Arm at {self._encoder.getPosition():1.1f}cm goal {self.getSetAngle():1.1f}cm")
            self._trackingAlert.set(True)

        # safe the motors if forward limit is hit
        if self.getForwardLimit():
            self._limitAlert.setText(f"Arm TOP HIT at {self._encoder.getPosition()}cm")
            self._limitAlert.set(True)
            self._encoder.setPosition(0)
            return

        if self.getReverseLimit() and self._curentGoal > 0:
            self._primaryMotor.set(0)
            self._limitAlert.setText(f"Arm BOTTOM HIT at {self._encoder.getPosition()}cm")
            self._limitAlert.set(True)
            return

        # clear the alert as we no longer are at the limit
        self._limitAlert.set(False)

        if self._disabled:
            self._primaryMotor.disable()
            return

        self._controller.setReference(self._curentGoal, self._primaryMotor.ControlType.kPosition)


    def getRotFromCm(self, cm : float):
        return cm * c.kEncoderFullRangeRot / c.kFullRangeDegrees;
    def getCmFromRot(self, rot:float):
        return rot * c.kFullRangeDegrees / c.kEncoderFullRangeRot



    def getForwardLimit(self) -> bool:
        """Returns if either motor controller has hit the forward limit switch
        Returns:
            bool: True if either motor controller has hit the forward limit switch
        """
        #TODO add soft limits?
        return self._primaryMotor.getForwardLimitSwitch().get()

    def getReverseLimit(self) -> bool:
        #TODO add soft limits?
        return self._primaryMotor.getReverseLimitSwitch().get()


    def setHeight(self, angleDeg: float) -> None:
        """
        Sets the height of the arm goal in meters from ground.
        """
        #0..-1 control
        angleDeg = -angleDeg
        # TODO add max and min set constraints to setters
        if self._disabled:
            self._disabled = False
            self._controller.reset(self._encoder.getDistance())

        angleDeg = angleDeg

        if angleDeg < 0:
            angleDeg = 0


        # cache goal for easy access and to use in periodic
        self._curentGoal = self.getRotFromCm(angleDeg)

    def getSetAngle(self) -> float:
        """
        Returns the current goal height of the arm in meters from ground.
        Returns:
            float: set heigh in degrees
        """
        return self.getCmFromRot(-self._curentGoal)

    def getAngleDeg(self) -> float:
        """Returns the current height of the arm in meters from ground.
        """
        return self.getCmFromRot(self._encoder.getPosition())

    def atGoal(self) -> bool:
        """
        Returns if the arm is at the goal height.
        """
        return  math.isclose(self._encoder.getVelocity(), 0.0, abs_tol=0.01) and math.isclose(self._encoder.getPosition(), self._curentGoal, abs_tol=0.1)

    def getError(self) -> float:
        """
        Returns the error in meters between the goal height and the current height.
        """
        return self.getCmFromRot(self._curentGoal - self._encoder.getPosition())

    def setIncrementalMove(self, deltaM: float) -> None:
        """
        Moves the arm by the delta in meters from the current goal height.
        """
        self.setHeight(self.getSetHeightM() + deltaM)

    def stopArm(self) -> None:
        """
        Stops the by setting current height to goal. This will maintain the current height.
        """
        self.setHeight(self.getHeightM())

    def disable(self) -> None:
        """
        Disables the elvators motors. The arm may fall under gravity.
        """
        self._disabled = True

    def getDisabled(self) -> bool:
        return self._disabled
