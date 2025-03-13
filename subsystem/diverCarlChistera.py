import wpilib
import math
import commands2
import rev
from constants import DiverCarlChisteraConsts as c
from constants import MechConsts as mc
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from subsystem.diverCarlElevator import DiverCarlElevator

class DiverCarlChistera(commands2.Subsystem):
    elevator: "DiverCarlElevator"

    armSafeAngleStart = 0.105 # movement arc


    def __init__(self, dt=0.02) -> None:
        super().__init__()
        self._dt = dt
        self.elevator = None
        self._primaryMotor = rev.SparkFlex(c.kMotorPrimaryCanId, rev.SparkLowLevel.MotorType.kBrushless)
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
        self._currentGoal = 0.0

        # setup telem
        self._limitAlert = wpilib.Alert("mechanism","Arm Limit Reached", wpilib.Alert.AlertType.kWarning)
        self._limitAlert.set(False)
        self._trackingAlert = wpilib.Alert("mechanism","Arm moving", wpilib.Alert.AlertType.kInfo)
        self._trackingAlert.set(False)

        wpilib.SmartDashboard.putNumber("Mech/Arm/PositionA", 0)
        wpilib.SmartDashboard.putNumber("Mech/Arm/SetA", 0)
        wpilib.SmartDashboard.putNumber("Mech/Arm/ReqA", 0)
        self._requestedGoal = 0

    def setElevator(self, elevator: "DiverCarlElevator") -> None:
        self.elevator = elevator


    def periodic(self) -> None:
        # update telemetry
        if self.atGoal():
            self._trackingAlert.setText(f"Arm stable at {self.getArc():1.1f}")
            self._trackingAlert.set(True)
        else:
            self._trackingAlert.setText(f"Arm at {self.getArc():1.1f} goal {self.getSetArc():1.1f}")
            self._trackingAlert.set(True)


        # safe the motors if forward limit is hit
        if self.getForwardLimit():
            self._limitAlert.setText(f"Arm TOP HIT at {self.getArc()}")
            self._limitAlert.set(True)
            self._encoder.setPosition(0)
            return

        if self.getForwardLimit() and self._currentGoal > 0:
            self._primaryMotor.set(0)
            self._limitAlert.setText(f"Arm BOTTOM HIT at {self.getArc()}")
            self._limitAlert.set(True)
            return

        # clear the alert as we no longer are at the limit
        self._limitAlert.set(False)

        if self._disabled:
            self._primaryMotor.disable()
            return

        calcGoal = self._currentGoal
        currHeight = 0
        if self.elevator is not None:
            currHeight = self.elevator.getPosition()
        #only allow arm > min safe when elevator is up
        if currHeight > mc.kElevatorSafeHeight: #arb 10 rotations for now
            if calcGoal < mc.kArmSafeAngleStart:
                calcGoal = mc.kArmSafeAngleStart
        #only allow arm < max safe when elevator is down
        else:
            if calcGoal > mc.kArmSafeAngleEnd:
                calcGoal = mc.kArmSafeAngleEnd

        #TODO add checks if elevator is moving keep in safe zone until position is reached

        wpilib.SmartDashboard.putNumber("Mech/Arm/PositionA", calcGoal)
        wpilib.SmartDashboard.putNumber("Mech/Arm/SetA", self.getArc())
        wpilib.SmartDashboard.putNumber("Mech/Arm/ReqA", self._currentGoal)

        self._controller.setReference(-calcGoal, self._primaryMotor.ControlType.kPosition, slot = rev.ClosedLoopSlot.kSlot0)
        self._requestedGoal = calcGoal


    def getRotFromArc(self, arc : float):
        return arc * c.kEncoderFullRangeRot / c.kFullRangeDegrees;
    def getArcFromRot(self, deg:float):
        return deg * c.kFullRangeDegrees / c.kEncoderFullRangeRot



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

    def setArc(self, arc: float) -> None:
        if self._disabled:
            self._disabled = False

        if arc < 0:
            arc = 0

        self._currentGoal = arc

    def getArc(self) -> float:
        return -self._encoder.getPosition()

    def getSetArc(self) -> float:
        return self._currentGoal

    def atGoal(self) -> bool:
        """
        Returns if the arm is at the goal height.
        """
        return  math.isclose(self._encoder.getVelocity(), 0.0, abs_tol=0.01) and math.isclose(self.getArc(), self._currentGoal, abs_tol=0.1)

    def getError(self) -> float:
        """
        Returns the error full arc between the goal height and the current height.
        """
        return self._currentGoal - self.getArc()

    def stopArm(self) -> None:
        """
        Stops the by setting current height to goal. This will maintain the current height.
        """
        self.setArc(self.getArc())

    def disable(self) -> None:
        """
        Disables the arm motors. The arm may fall under gravity.
        """
        self._disabled = True

    def getDisabled(self) -> bool:
        return self._disabled
