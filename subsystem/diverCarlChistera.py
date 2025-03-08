import wpilib
import wpimath
import commands2
import rev
import wpimath.trajectory
import wpimath.controller
import wpimath.units
import math
from constants import DiverCarlChisteraConsts as c
# plan to use 25:1 gear ratio
# drum TBD
# 2 motors one will run opposite direction


class DiverCarlChistera(commands2.Subsystem):

    def __init__(self, dt=0.02) -> None:
        self._dt = dt
        self._primaryMotor = rev.SparkMax(c.kMotorCanId, rev.SparkLowLevel.MotorType.kBrushless)
        # setup primary
        motorConfig = rev.SparkBaseConfig()
        motorConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        motorConfig.inverted(c.kMotorInverted)

        #enable soft forward limits
        softLimit = rev.SoftLimitConfig()
        softLimit.forwardSoftLimit(c.kSoftLimits["forwardLimit"])
        softLimit.forwardSoftLimitEnabled(c.kSoftLimits["forward"])
        softLimit.reverseSoftLimit(c.kSoftLimits["reverseLimit"])
        softLimit.reverseSoftLimitEnabled(c.kSoftLimits["reverse"])

        motorConfig.apply(softLimit)

        hardLimits = rev.LimitSwitchConfig()
        hardLimits.forwardLimitSwitchEnabled(c.kLimits["forward"])
        hardLimits.forwardLimitSwitchType(c.kLimits["forwardType"])
        hardLimits.reverseLimitSwitchEnabled(c.kLimits["reverse"])
        hardLimits.reverseLimitSwitchType(c.kLimits["reverseType"])
        motorConfig.apply(hardLimits)


        self._encoder = self._primaryMotor.getEncoder()
        encConfig = rev.EncoderConfig()
        encConfig.positionConversionFactor(1/c.kGearRatio * math.pi * 2)
        encConfig.velocityConversionFactor(1/c.kGearRatio * math.pi * 2)

        motorConfig.apply(encConfig)
        self._primaryMotor.configure(motorConfig, rev.SparkBase.ResetMode.kNoResetSafeParameters , rev.SparkBase.PersistMode.kPersistParameters)
        # set follower like primary except inverted

        self._constraints = wpimath.trajectory.TrapezoidProfile.Constraints(c.kMaxVelRPS, c.kMaxAccelRPSS)
        self._controller = wpimath.controller.ProfiledPIDController(*c.kPid, self._constraints, self._dt)

        #TODO adjust tolerance. Currenlty 4% of 360 roation
        self._controller.setTolerance(2*math.pi * 0.04)


        self._disabled = False
        self._curentGoal = 0.0

        # setup telem
        self._limitAlert = wpilib.Alert("mechanism","Arm Limit Reached", wpilib.Alert.AlertType.kWarning)
        self._limitAlert.set(False)
        self._trackingAlert = wpilib.Alert("mechanism","Arm moving", wpilib.Alert.AlertType.kInfo)
        self._trackingAlert.set(False)

    def periodic(self) -> None:
        # update telemtry
        if self._controller.atGoal():
            self._trackingAlert.setText(f"Arm stable at {self._encoder.getDistance():1.1f}m")
            self._trackingAlert.set(True)
        else:
            self._trackingAlert.setText(f"arm at {self._encoder.getDistance():1.1f}m goal {self.getSetHeightM():1.1f}m")
            self._trackingAlert.set(True)

        if self.getReverseLimit():
            self._encoder.reset()

        # safe the motors if forward limit is hit
        if self.getForwardLimit() and self._motors.get() > 0:
            self._motors.set(0)
            self._limitAlert.setText(f"Elevator TOP HIT at {self._encoder.getDistance()}m")
            self._limitAlert.set(True)
            return

        if self.getReverseLimit() and self._motors.get() < 0:
            self._motors.set(0)
            self._limitAlert.setText(f"Elevator BOTTOM HIT at {self._encoder.getDistance()}m")
            self._limitAlert.set(True)
            return

        # clear the alert as we no longer are at the limit
        self._limitAlert.set(False)

        if self._disabled:
            self._motors.set(0)
            return

        output = self._controller.calculate(self._encoder.getDistance());
        if output > 0.1:
            output = 0.1
        if output < -0.1:
            output = -0.1


        self._motors.set(output)

    def getForwardLimit(self) -> bool:
        """Returns if either motor controller has hit the forward limit switch
        Returns:
            bool: True if either motor controller has hit the forward limit switch
        """
        return self._primaryMotor.getForwardLimitSwitch().get()

    def getReverseLimit(self) -> bool:
        return self._primaryMotor.getReverseLimitSwitch().get()


    def setHeight(self, heightM: float) -> None:
        """
        Sets the height of the elevator goal in meters from ground.
        """
        # TODO add max and min set constraints to setters
        if self._disabled:
            self._disabled = False
            self._controller.reset(self._encoder.getDistance())

        heightM = heightM - self._heightDelta

        if heightM < 0:
            heightM = 0

        # cache goal for easy access
        self._curentGoal = heightM
        self._controller.setGoal(heightM)

    def getSetHeightM(self) -> float:
        """
        Returns the current goal height of the elevator in meters from ground.
        Returns:
            float: set heigh in meters
        """
        return self._curentGoal+self._heightDelta

    def getHeightM(self) -> float:
        """Returns the current height of the elevator in meters from ground.
        """
        return self._encoder.getDistance()

    def atGoal(self) -> bool:
        """
        Returns if the elevator is at the goal height.
        """
        return self._controller.atGoal()

    def getError(self) -> float:
        """
        Returns the error in meters between the goal height and the current height.
        """
        return self._controller.getPositionError()

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
