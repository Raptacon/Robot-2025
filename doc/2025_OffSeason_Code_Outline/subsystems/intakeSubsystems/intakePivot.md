from wpimath.controller import PIDController, ArmFeedforward
from wpimath.trajectory import TrapezoidProfile
#ccw rotation of the motor(up positive, down negative)

class intakePivot:
    def __init__(self, ):
        #Constants
        #Will most likely change in the future
        #seperate file
        self.kMinSpeed: float = 0
        self.kMaxSpeed: float = 90
        self.kMinAcceleration: float = 0
        self.kMaxAcceleration: float = 150
        self.kMinPos: float = 0
        self.kMaxPos: float = 135
        self.kPivotPValue: float = 1
        self.kPivotIValue: float = 2
        self.kPivotDValue: float = 3
        self.kPivotStaticFriction: float = 1
        self.kPivotGravity: float = 2
        self.kPivotVelocity: float = 3
        self.kPivotTolerance: float = 1
        self.kSoftConstraint: float = 27 

        self.trapezoidConstraints = TrapezoidProfile.Constraints(self.kMaxSpeed, self.kMaxAcceleration)
        self.previousState = TrapezoidProfile.State(0, self.kMinSpeed)

        self.trapezoidProfile = TrapezoidProfile(self.trapezoidConstraints)
        
        self.pid = PIDController(self.kPivotPValue, self.kPivotIValue, self.kPivotDValue)
        self.pid.setTolerance(self.kPivotTolerance)

        self.feedForward = ArmFeedforward(self.kPivotStaticFriction, self.kPivotGravity, self.kPivotVelocity)

        self.limitSwitchUpper = limitSwitch(0?)
        self.limitSwitchUpper = limitSwitch(1?)

        self.IntakePivotMotor = motor(20?)

        self.intakePivotMotorEncoder = motorEncoder(20?)
        self.intakePivotMotorEncoder.setPosition(self.kMaxPos)

        self.setGoal(0)

    def setGoal(self, goal: float) -> None:
        self.goal = goal
        self.updatePreviousState()

    def setMotorSpeed(self, motorSpeed: float) -> None:
        self.motorSpeed = motorSpeed

    def softLimits(self):
        if (self.intakePivotMotorEncoder.getVelocity() > 0) and (self.getRotaionalPosition() >= (self.kMaxPos - self.kSoftConstraint)):
            self.IntakePivotMotor.set(0.1)
        if (self.intakePivotMotorEncoder.getVelocity() < 0) and (self.getRotaionalPosition() <= (self.kMinPos + self.kSoftConstraint)):
            self.IntakePivotMotor.set(-0.1)

    def hardConstraints(self) -> None:
        if self.limitSwitchUpper or self.limitSwitchUpper:
            self.setMotorSpeed(0)
        if self.getRotaionalPosition <= self.kMinPos or self.getRotaionalPosition >= self.kMaxPos:
            self.setMotorSpeed(0)
        if self.limitSwitchUpper: 
            self.intakePivotMotorEncoder.setPosition(self.kMaxPos)
        if self.limitSwitchLower:
            self.intakePivotMotorEncoder.setPosition(self.kMinPos)

    def getRotaionalPosition(self) -> float:
        return self.intakePivotMotorEncoder.getPosition() % 360

    def updatePreviousState(self):
        self.previousState = TrapezoidProfile.State(self.getRotaionalPosition(), self.intakePivotMotorEncoder.getVelocity())

    def turnToGoal(self) -> bool:
        self.previousState = self.trapezoidProfile.calculate(0.02, self.previousState, TrapezoidProfile.State(self.goal, 0))
        self.feedForwardVoltage = self.feedForward.calculate(
            self.getRotationalPosition(), self.intakePivotMotorEncoder.getVelocity(), self.previousState.velocity
        )
        self.feedBackVoltage = self.pid.calculate(self.getRotaionalPosition(), self.goal)
        self.IntakePivotMotor.setVoltage(self.feedBackVoltage + self.feedForwardVoltage)
        return self.pid.atSetpoint()

    def periodic(self) -> None:
        self.softLimits()
        self.hardConstraints()
