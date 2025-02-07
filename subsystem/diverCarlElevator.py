import wpilib
import wpimath
import commands2
import rev
import wpimath.trajectory
import wpimath.controller
import wpimath.units

#plan to use 25:1 gear ratio
#drum TBD
# 2 motors one will run opposite direction


class DiverCarlElevator(commands2.SubsystemBase):
    kMotorPrimaryCanId = 10
    kMotorFollowerCanId = 11
    kEncoderPinA = 9
    kEncoderPinB = 8
    kMinHeightM = 0
    kMaxHeightM = 1.5
    kMaxVelMPS = 0.75
    kMaxAccelMPSS = 0.2
    kMotorPrimaryInverted = False

    def __init__(self, dt=0.02 ) -> None:
        self.dt = dt
        self.primaryMotor = rev.SparkMax(self.kMotorPrimaryCanId, rev.SparkLowLevel.MotorType.kBrushless)
        self.followerMotor = rev.SparkMax(self.kMotorFollowerCanId, rev.SparkLowLevel.MotorType.kBrushless)
        #setup primary
        motorConfig = rev.SparkBaseConfig()
        motorConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        motorConfig.inverted(self.kMotorPrimaryInverted)
        self.primaryMotor.configure(motorConfig, rev.SparkBase.ResetMode.kNoResetSafeParameters , rev.SparkBase.PersistMode.kPersistParameters)
        #set follower like primary except inverted
        motorConfig.inverted(not self.kMotorPrimaryInverted)
        self.followerMotor.configure(motorConfig, rev.SparkBase.ResetMode.kNoResetSafeParameters , rev.SparkBase.PersistMode.kPersistParameters)
        
        self.motors = wpilib.MotorControllerGroup(self.primaryMotor, self.followerMotor)

        self.encoder = wpilib.Encoder(self.kEncoderPinA, self.kEncoderPinB)
        #0.1mm to meters
        self.encoder.setDistancePerPulse(0.1 / 1000)

        self.constraints = wpimath.trajectory.TrapezoidProfile.Constraints(self.kMaxVelMPS, self.kMaxAccelMPSS)
        self.controller = wpimath.controller.ProfiledPIDController(1.3, 0, 0.7, self.constraints, self.dt)
        self.controller.setTolerance(0.01)

        
    def periodic(self) -> None:
        #safe the motors if forward limit is hit
        if self.primaryMotor.getForwardLimitSwitch().get() and self.motors.get() > 0:
            self.motors.set(0)
        if self.primaryMotor.getReverseLimitSwitch().get() and self.motors.get() > 0:
            self.motors.set(0)        

        self.motors.set(self.controller.calculate(self.encoder.getDistance()))

        

    def setHeight(self, height: float) -> None:
        self.controller.setGoal(height)
    
    def getSetHeight(self) -> float:
        return self.controller.getGoal()

    def getHeight(self) -> float:
        return self.encoder.getDistance()
    
    def atGoal(self) -> bool:
        return self.controller.atGoal()
    
    def getError(self) -> float:
        return self.controller.getPositionError()
    
