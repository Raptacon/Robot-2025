# Native imports
import math

# Internal imports
from constants import DiverCarlElevatorConsts as c
from constants import MechConsts as mc

# Third-party imports
import commands2
import rev
from wpilib import Timer
from wpimath.controller import ElevatorFeedforward
from wpimath.trajectory import TrapezoidProfile

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from subsystem.diverCarlChistera import DiverCarlChistera


class DiverCarlElevator(commands2.Subsystem):
    """ """
    constants = c
    _arm : "DiverCarlChistera"

    def __init__(
        self, height_at_zero: float = c.kHeightAtZeroCm, update_period: float = 0.05
    ) -> None:
        """ """
        super().__init__()

        # Save starting configurations
        self.height_at_zero = height_at_zero
        self.update_period = update_period

        # Set up objects for controlling the motor
        self.timer = Timer()
        self.motor = rev.SparkFlex(
            c.kMotorCanId, rev.SparkLowLevel.MotorType.kBrushless
        )
        self.encoder = self.motor.getEncoder()
        self.motor_pid = self.motor.getClosedLoopController()
        self.profilerUp = TrapezoidProfile(
            TrapezoidProfile.Constraints(*c.kTrapezoidProfileUp)
        )
        self.profilerDown = TrapezoidProfile(
            TrapezoidProfile.Constraints(*c.kTrapezoidProfileDown)
        )
        self.feedforward = ElevatorFeedforward(*c.kFeedforward, self.update_period)

        # Configure motor
        self.configureMotor()
        self.encoder.setPosition(0)

        # Instantiate elevator state variables for telemetry
        self.current_goal_height = self.height_at_zero
        self.current_goal_height_above_zero = 0
        self.last_profiler_state = TrapezoidProfile.State(0, 0)
        self.updateSensorRecordings()

    def configureMotor(self) -> None:
        """ """
        motor_config = rev.SparkBaseConfig()

        # Set overall configurations
        (
            motor_config.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
            .inverted(c.kMotorInverted)
            .smartCurrentLimit(c.kCurrentLimitAmps)
        )

        # Configure soft limits (software-enforced elevator range of motion limits)
        (
            motor_config.softLimit.forwardSoftLimit(c.kSoftLimits["forwardLimit"])
            .forwardSoftLimitEnabled(c.kSoftLimits["forward"])
            .reverseSoftLimit(c.kSoftLimits["reverseLimit"])
            .reverseSoftLimitEnabled(c.kSoftLimits["reverse"])
        )

        # Configure hard limits (hardware triggers to stop motor execution)
        (
            motor_config.limitSwitch.forwardLimitSwitchEnabled(c.kLimits["forward"])
            .forwardLimitSwitchType(c.kLimits["forwardType"])
            .reverseLimitSwitchEnabled(c.kLimits["reverse"])
            .reverseLimitSwitchType(c.kLimits["reverseType"])
        )

        # Set conversion factors to turn motor units into centimeters
        (
            motor_config.encoder.positionConversionFactor(
                c.kMaxHeightAboveZeroCm / c.kRotationsToMaxHeight
            ).velocityConversionFactor(
                (c.kMaxHeightAboveZeroCm / c.kRotationsToMaxHeight) / 60
            )
        )

        # Set up the PID controller
        (
            motor_config.closedLoop.pid(*c.kPid0)
            .outputRange(*c.kMaxOutRange0)
            .setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
        )

        # Apply the configuration and burn to the SparkFlex's flash memory
        self.motor.configure(
            motor_config,
            rev.SparkBase.ResetMode.kNoResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters,
        )

    def resetProfilerState(self) -> None:
        """ """
        self.last_profiler_state = TrapezoidProfile.State(
            self.current_height_above_zero, self.motor_velocity
        )

    def validateGoalHeight(self) -> None:
        """ """
        if self.current_goal_height < self.height_at_zero:
            self.current_goal_height = self.height_at_zero
        if self.current_goal_height > (c.kMaxHeightAboveZeroCm + self.height_at_zero):
            self.current_goal_height = c.kMaxHeightAboveZeroCm + self.height_at_zero

        if self.current_goal_height_above_zero < 0:
            self.current_goal_height_above_zero = 0
        if self.current_goal_height_above_zero > c.kMaxHeightAboveZeroCm:
            self.current_goal_height_above_zero = c.kMaxHeightAboveZeroCm

    def setArm(self, arm: "DiverCarlChistera") -> None:
        """ """
        self._arm = arm


    def setGoalHeight(self, height_cm: float) -> None:
        """ """
        self.current_goal_height = height_cm
        self.current_goal_height_above_zero = (
            self.current_goal_height - self.height_at_zero
        )
        self.validateGoalHeight()

    def getPosition(self) -> float:
        """Returns position in motor rotations"""
        return self.encoder.getPosition()

    def incrementGoalHeight(self, height_increment_cm: float) -> None:
        """ """
        self.current_goal_height = self.current_goal_height + height_increment_cm
        self.current_goal_height_above_zero = (
            self.current_goal_height_above_zero + height_increment_cm
        )
        self.validateGoalHeight()

    def goToGoalHeight(self) -> None:
        """
        Sets the height of the elevator goal in centimeters from the ground.
        """
        profiler_use = self.profilerDown
        if self.current_goal_height_above_zero > self.last_profiler_state.position:
            profiler_use = self.profilerUp

        self.last_profiler_state = profiler_use.calculate(
            self.update_period,
            self.last_profiler_state,
            TrapezoidProfile.State(self.current_goal_height_above_zero, 0),
        )

        # Protect the arm from moving into unsafe position while arm is not in the correct position
        currArmArc = 0
        if self._arm is not None:
            currArmArc = self._arm.getArc()

        currPos = self.last_profiler_state.position
        #if arm is near parked, max height = mc.kElevatorSafeHeight
        if currArmArc < mc.kArmSafeAngleStart:
            if currPos > mc.kElevatorSafeHeight:
                currPos = mc.kElevatorSafeHeight
        #if arm is in safe zone, any height is valid


        self.motor_pid.setReference(
            currPos,
            rev.SparkLowLevel.ControlType.kPosition,
            rev.ClosedLoopSlot.kSlot0,
            self.feedforward.calculate(
                self.motor_velocity, self.last_profiler_state.velocity
            ),
            rev.SparkClosedLoopController.ArbFFUnits.kVoltage,
        )

    def checkIfAtGoalHeight(self) -> bool:
        """ """
        return math.isclose(
            self.encoder.getVelocity(), 0.0, abs_tol=0.1
        ) and math.isclose(
            self.encoder.getPosition(), self.current_goal_height_above_zero, abs_tol=1
        )

    def manualControl(self, velocity_percentage: float) -> None:
        """ """
        desired_velocity = velocity_percentage * (c.kTrapezoidProfileUp[0] / 4)
        if desired_velocity < 0:
            desired_velocity = velocity_percentage * (c.kTrapezoidProfileDown[0] / 3)
        if self.at_bottom_limit or self.at_top_limit:
            desired_velocity = 0
        self.motor.setVoltage(self.feedforward.calculate(desired_velocity))

    def updateSensorRecordings(self) -> None:
        """ """
        self.current_height_above_zero = self.encoder.getPosition()
        self.current_height = self.height_at_zero + self.current_height_above_zero
        self.at_top_limit = self.motor.getForwardLimitSwitch().get()
        self.at_bottom_limit = self.motor.getReverseLimitSwitch().get()
        self.motor_current = self.motor.getOutputCurrent()
        self.motor_output = self.motor.getAppliedOutput()
        self.motor_velocity = self.encoder.getVelocity()
        self.at_goal = self.checkIfAtGoalHeight()
        self.error_from_goal = (
            self.current_height_above_zero - self.current_goal_height_above_zero
        )

    def periodic(self) -> None:
        """ """
        self.updateSensorRecordings()

        if self.at_bottom_limit:
            self.encoder.setPosition(0)

        if (self.at_top_limit and self.motor.get() > 0) or (
            self.at_bottom_limit and self.motor.get() < 0
        ):
            self.motor.set(0)
            return

    def disable(self) -> None:
        """ """
        self.motor.disable()
