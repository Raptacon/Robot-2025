import wpilib
import wpimath
import commands2
import rev
import wpimath.trajectory
import wpimath.controller
import wpimath.units
import math
from constants import DiverCarlElevatorConsts as c
# plan to use 25:1 gear ratio
# drum TBD
# 2 motors one will run opposite direction


class DiverCarlElevator(commands2.Subsystem):

    def __init__(self, elevatorHeightDelta: float = c.kMechDeltaHeightM, dt=0.02) -> None:
        self._dt = dt
        self._heightDelta = elevatorHeightDelta
        self._primaryMotor = rev.SparkMax(c.kMotorPrimaryCanId, rev.SparkLowLevel.MotorType.kBrushless)
        # setup primary
        motorConfig = rev.SparkBaseConfig()
        motorConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        motorConfig.inverted(c.kMotorPrimaryInverted)
        motorConfig.encoder.positionConversionFactor((math.pi * 0.1016) / 25)
        self._primaryMotor.configure(motorConfig, rev.SparkBase.ResetMode.kNoResetSafeParameters , rev.SparkBase.PersistMode.kPersistParameters)

        self._encoder = self._primaryMotor.getEncoder()

        self._constraints = wpimath.trajectory.TrapezoidProfile.Constraints(c.kMaxVelMPS, c.kMaxAccelMPSS)
        self._controller = wpimath.controller.ProfiledPIDController(*c.kPid, self._constraints, self._dt)

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
        wpilib.SmartDashboard.putNumber("logged_e_height", self.getHeightM())
        wpilib.SmartDashboard.putBoolean("at_goal", self.atGoal())
        wpilib.SmartDashboard.putNumber("pos_error", self.getError())

        # update telemtry
        if self._controller.atGoal():
            self._trackingAlert.setText(f"Elevator stable at {self._encoder.getPosition():1.1f}m")
            self._trackingAlert.set(True)
        else:
            self._trackingAlert.setText(f"Elevator at {self._encoder.getPosition():1.1f}m goal {self.getSetHeightM():1.1f}m")
            self._trackingAlert.set(True)

        if self.getReverseLimit():
            self._encoder.setPosition(0)

        # safe the motors if forward limit is hit
        if self.getForwardLimit() and self._primaryMotor.get() > 0:
            self._primaryMotor.set(0)
            self._limitAlert.setText(f"Elevator TOP HIT at {self._encoder.getPosition()}m")
            self._limitAlert.set(True)
            return

        if self.getReverseLimit() and self._primaryMotor.get() < 0:
            self._primaryMotor.set(0)
            self._limitAlert.setText(f"Elevator BOTTOM HIT at {self._encoder.getPosition()}m")
            self._limitAlert.set(True)
            return

        # clear the alert as we no longer are at the limit
        self._limitAlert.set(False)

        if self._disabled:
            self._primaryMotor.set(0)
            return

        output = self._controller.calculate(self._encoder.getPosition())
        self._primaryMotor.set(output)

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
            self._controller.reset(self._encoder.getPosition())

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
        return self._encoder.getPosition()

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
