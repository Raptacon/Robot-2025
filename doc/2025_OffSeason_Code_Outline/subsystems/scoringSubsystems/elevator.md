from wpimath.controller import PIDController, ElevatorFeedforward
from wpimath.trajectory import TrapezoidProfile

class elevator():
    def __init__():
        #consts, will be in a seperate constants file
        self.kElevatorMaxSpeed: float = 90
        self.kElevatorMinSpeed: float = 0
        self.kElevatorMaxAcceleration: float = 150
        self.kElevatorMinAcceleration: float = 0
        self.kElevatorMinPos: float = 0 # cm
        self.kElevatorMaxPos: float = 180 # cm
        self.kElevatorPValue: float = 1
        self.kElevatorIValue: float = 2
        self.kElevatorDValue: float = 3
        self.kElevatorStaticFriction: float = 1
        self.kElevatorGravity: float = 2
        self.kElevatorVelocity: float = 3
        self.kElevatorTolerance: float = 1
        self.kElevatorSoftConstraint: float = # need to do math

        self.trapezoidConstraints = TrapezoidProfile.Constraints(self.kElevatorMaxSpeed, self.kElevatorMaxAcceleration)
        self.previousState = TrapezoidProfile.State(0, self.kElevatorMinSpeed)

        self.trapezoidProfile = TrapezoidProfile(self.trapezoidConstraints)

        self.pid = PIDController(self.kElevatorPValue, self.kElevatorIValue, self.kElevatorDValue)
        self.pid.setTolerance(self.kElevatorTolerance)

        self.feedForward = ElevatorFeedforward(self.kElevatorStaticFriction, self.kElevatorGravity, self.kElevatorVelocity)

        self.limitSwitchTop = limitSwitch(2?)
        self.limitSwitchBottom = limitSwitch(3?)

        self.elevatorMotorLeft = motor(30?)
        self.elevatorMotorRight = motor(31?)

        self.elevatorMotorEncoderLeft = motor(30?)
        self.elevatorMotorEncoderRight = motor(31?)
        self.elevatorMotorEncoderLeft.setPosition(self.kElevatorMinPos)
        self.elevatorMotorEncoderRight.setPosition(self.kElevatorMinPos)

        self.setGoal(0)

    def setGoal(self, goal: float):
        self.goal = goal

    def setMotorSpeed(self, motorSpeed: float):
        self.motorSpeed = motorSpeed
    
    def getPosition(self):
        return self.elevatorMotorEncoder.getPosition()

    def softLimits(self):
        if (self.elevatorMotorEncoder.getVelocity() > 0) and (self.getPosition() >= (self.kElevatorMaxPos - self.kElevatorSoftConstraint)):
            self.elevatorMotor.set(0.1)
        if (self.elevatorMotorEncoder.getVelocity() < 0) and (self.getPosition() <= (self.kElevatorMinPos + self.kElevatorSoftConstraint)):
            self.elevatorMotor.set(-0.1)

    def hardLimits(sefl):
        if self.limitSwitchUpper or self.limitSwitchUpper:
            self.setMotorSpeed(0)
        if self.getPosition <= self.kElevatorMinPos or self.getPosition >= self.kElevatorMaxPos:
            self.setMotorSpeed(0)
        if self.limitSwitchTop: 
            self.elevatorMotorEncoder.setPosition(self.kElevatorMaxPos)
        if self.limitSwitchLower:
            self.elevatorMotorEncoder.setPosition(self.kElevatorMinPos)

    def updatePreviousState(self):
        self.TrapezoidProfile.State(self.getPosition(), self.elevatorMotorEncoder.getVelocity())

    def elevate() -> bool:
        self.previousState = self.trapezoidProfile.calculate(0.02, self.previousState, TrapezoidProfile.State(self.goal, 0))
        self.feedForwardVoltage = self.feedForward.calculate(
            self.getPosition(), self.elevatorMotorEncoder.getVelocity(), self.previousState.velocity
        )
        self.feedBackVoltage = self.pid.calculate(self.getPosition(), self.goal)
        self.elevatorMotor.setVoltage(self.feedBackVoltage + self.feedForwardVoltage)
        return self.pid.atSetpoint()

    def stopElevatorMotors(self):
        self.elevatorMotorLeft.set(0)
        self.elevatorMotorRight.set(0)

    def periodic():
        self.softLimits()
        self.hardLimits()
