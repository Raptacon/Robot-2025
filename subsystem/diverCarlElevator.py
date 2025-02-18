import wpilib
import wpimath
import commands2
import rev
import wpimath.trajectory
import wpimath.controller
import wpimath.units

# plan to use 25:1 gear ratio
# drum TBD
# 2 motors one will run opposite direction


class DiverCarlElevator(commands2.Subsystem):
    kMotorPrimaryCanId = 10
    kMotorFollowerCanId = 11
    kEncoderPinA = 9
    kEncoderPinB = 8
    kMinHeightM = 0
    kMaxHeightM = 1.5
    kMaxVelMPS = 0.75
    kMaxAccelMPSS = 0.2
    kMotorPrimaryInverted = False

    def __init__(self, dt=0.02) -> None:
        self._dt = dt
        self._primaryMotor = rev.SparkMax(self.kMotorPrimaryCanId, rev.SparkLowLevel.MotorType.kBrushless)
        self._followerMotor = rev.SparkMax(self.kMotorFollowerCanId, rev.SparkLowLevel.MotorType.kBrushless)
        # setup primary
        motorConfig = rev.SparkBaseConfig()
        motorConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        motorConfig.inverted(self.kMotorPrimaryInverted)
        self._primaryMotor.configure(motorConfig, rev.SparkBase.ResetMode.kNoResetSafeParameters , rev.SparkBase.PersistMode.kPersistParameters)
        # set follower like primary except inverted
        motorConfig.inverted(not self.kMotorPrimaryInverted)
        self._followerMotor.configure(motorConfig, rev.SparkBase.ResetMode.kNoResetSafeParameters , rev.SparkBase.PersistMode.kPersistParameters)

        self._motors = wpilib.MotorControllerGroup(self._primaryMotor, self._followerMotor)

        self._encoder = wpilib.Encoder(self.kEncoderPinA, self.kEncoderPinB)
        # 0.1mm to meters
        self._encoder.setDistancePerPulse(0.1 / 1000)

        self._constraints = wpimath.trajectory.TrapezoidProfile.Constraints(self.kMaxVelMPS, self.kMaxAccelMPSS)
        self._controller = wpimath.controller.ProfiledPIDController(1.3, 0, 0.7, self._constraints, self._dt)

        # TODO adjust tolerance. Currently set to 1cm (0.01 m)
        self._controller.setTolerance(0.01)

        self._disabled = False
        self._curentGoal = 0.0

        # setup telem
        self._limitAlert = wpilib.Alert("mechanism","Elevator Limit Reached", wpilib.Alert.AlertType.kWarning)
        self._limitAlert.set(False)
        self._trackingAlert = wpilib.Alert("mechanism","Elevator moving", wpilib.Alert.AlertType.kInfo)
        self._trackingAlert.set(False)

    def periodic(self) -> None:
        # update telemtry
        if self._controller.atGoal():
            self._trackingAlert.setText(f"Elevator stable at {self._encoder.getDistance():1.1f}m")
            self._trackingAlert.set(True)
        else:
            self._trackingAlert.setText(f"Elevator at {self._encoder.getDistance():1.1f}m goal {self.getSetHeight():1.1f}m")
            self._trackingAlert.set(True)

        # safe the motors if forward limit is hit
        if self._primaryMotor.getForwardLimitSwitch().get() and self._motors.get() > 0:
            self._motors.set(0)
            self._limitAlert.setText(f"Elevator TOP HIT at {self._encoder.getDistance()}m")
            self._limitAlert.set(True)
            return

        if self._primaryMotor.getReverseLimitSwitch().get() and self._motors.get() < 0:
            self._motors.set(0)
            self._limitAlert.setText(f"Elevator BOTTOM HIT at {self._encoder.getDistance()}m")
            self._limitAlert.set(True)
            return

        # clear the alert as we no longer are at the limit
        self._limitAlert.set(False)

        if self._disabled:
            self._motors.set(0)
            return

        output = self._controller.calculate(self._encoder.getDistance())
        if output > 0.1:
            output = 0.1
        if output < -0.1:
            output = -0.1


        self._motors.set(output)



    def setHeight(self, height: float) -> None:
        # TODO add max and min set constraints to setters
        if self._disabled:
            self._disabled = False
            self._controller.reset(self._encoder.getDistance())

        # cache goal for easy access
        self._curentGoal = height
        self._controller.setGoal(height)

    def getSetHeight(self) -> float:
        return self._curentGoal

    def getHeight(self) -> float:
        return self._encoder.getDistance()

    def atGoal(self) -> bool:
        return self._controller.atGoal()

    def getError(self) -> float:
        return self._controller.getPositionError()

    def setIncrementalMove(self, delta: float) -> None:
        self.setHeight(self.getSetHeight() + delta)

    def stopElevator(self) -> None:
        self.setHeight(self.getHeight())

    def disable(self) -> None:
        self._disabled = True
