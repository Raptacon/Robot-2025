import math
import wpilib

class SteerController ():
    """Controller for sterring a swerve drive module. Cancoder and FX500 only"""
    kEncoderResetIterations = 500
    kEncoderResetMaxAngVel = math.radians(0.5)

    def __init__(self, swerveModule):
        self.module = swerveModule
        self.referenceAngle = 0.0
        self.resetIteration = 0


    def getReferenceAngle(self):
        """Gets current set angle in radians"""
        return self.referenceAngleRadians


    def setReferenceAngle(self, referenceAngle : float):
        '''https://github.com/SwerveDriveSpecialties/swerve-lib/blob/f6f4de65808d468ed01cc5ca39bf322383838fcd/src/main/java/com/swervedrivespecialties/swervelib/ctre/Falcon500SteerControllerFactoryBuilder.java#L181
           takes -180-180
        '''
        motor = self.module.getSteerMotor()
        #motorEncoderVelocityCoefficient = self.module.getSteerSensorVelocityCoefficient() #removed due to code below being removed
        currentAngle = self.module.getSteerAngle() # * motorEncoderPositionCoefficient

        wpilib.SmartDashboard.putNumber(f"{self.module.steerId}Steer goal", referenceAngle)
        wpilib.SmartDashboard.putNumber(f"{self.module.steerId}Steer curr", currentAngle)

        self.speed = self.module.steerPIDController.calculate(currentAngle, referenceAngle)
        motor.set(self.speed)
        self.referenceAngle = referenceAngle

