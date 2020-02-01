# -*- coding: utf-8 -*-

import rev
import ctre

import logging
log = logging.getLogger("console") #These logs were set up for testing, should not be persistent, please delete if you see these and I forgot
log.setLevel(logging.DEBUG)

def createMotor(motorDescp, motors = {}):
    '''This is where all motors are set up'''
    if motorDescp['type'] == 'CANTalon':
        #if we want to use the built in encoder set it here
        if('pid' in motorDescp) and motorDescp['pid'] != None:
            motor = WPI_TalonSRXFeedback(motorDescp)
            motor.setupPid()
        else:
            motor = ctre.WPI_TalonSRX(motorDescp['channel'])
        motors[str(motorDescp['channel'])] = motor

    elif motorDescp['type'] == 'CANTalonFollower':
        motor =ctre.WPI_TalonSRX(motorDescp['channel'])
        motor.set(mode = ctre.ControlMode.Follower, demand0 = motorDescp['masterChannel'])
        motors[str(motorDescp['channel'])] = motor

    if motorDescp['type'] == 'CANTalonFX':
        if('pid' in motorDescp) and motorDescp['pid'] != None:
            motor = WPI_TalonFXFeedback(motorDescp)
            motor.setupPid()
        else:
            motor = ctre.WPI_TalonFX(motorDescp['channel'])

    elif motorDescp['type'] == 'SparkMax':
        '''This is where SparkMax motor controllers are set up'''
        if 'pid' in motorDescp and motorDescp['pid'] != None:
            motor = SparkMaxFeedback(motorDescp, motors)
            motor.setupPid()
        else:
            motor = SparkMaxFeedback(motorDescp, motors)
        motors[str(motorDescp['channel'])] = motor

    elif motorDescp['type'] == 'SparkMaxFollower':
        '''This is where SparkMax followers are set up
        For masterChannel, use a motor object. MASTER MUST BE A "CANSparkMax"  '''
        motor = SparkMaxFeedback(motorDescp, motors)
        motor.follow(motors.get(str(motorDescp['masterChannel'])), motorDescp['inverted'])

    else:
        print("Unknown Motor")

    if 'inverted' in motorDescp:
        motor.setInverted(motorDescp['inverted'])

    if 'currentLimits' in motorDescp:
        currentLimits = motorDescp['currentLimits']
        absMax = currentLimits['absMax']
        absMaxTimeMs = currentLimits['absMaxTimeMs']
        nominalMaxCurrent = currentLimits['maxNominal']
        motor.configPeakCurrentLimit(absMax,10)
        motor.configPeakCurrentDuration(absMaxTimeMs,10)
        motor.configContinuousCurrentLimit(nominalMaxCurrent,10)
        motor.enableCurrentLimit(True)

    if 'rampRate' in motorDescp:
        motor.configOpenLoopRamp(motorDescp['rampRate'],10)

    return motor

class WPI_TalonSRXFeedback(ctre.WPI_TalonSRX):
    def __init__(self, motorDescription):
        ctre.WPI_TalonSRX.__init__(self,motorDescription['channel'])
        self.motorDescription = motorDescription
        self.pid = None

    def setupPid(self,motorDescription = None):
        if not motorDescription:
            motorDescription = self.motorDescription
        if not 'pid' in self.motorDescription:
            print("Motor channel %d has no PID"%(self.motorDescription['channel']))
            return
        self.pid = self.motorDescription['pid']
        self.controlType = self.pid['controlType']
        if self.controlType == "Position":
            self.controlType = ctre.ControlMode.Position
        elif self.controlType == "Velocity":
            self.controlType = ctre.ControlMode.Velocity
        
        self.configSelectedFeedbackSensor(ctre.FeedbackDevice(self.pid['feedbackDevice']), 0, 10)
        self.setSensorPhase(self.pid['sensorPhase'])
        self.pidControlType = self.pid['controlType']
        self.kPreScale = self.pid['kPreScale']

        #/* set the peak, nominal outputs, and deadband */
        self.configNominalOutputForward(0, 10)
        self.configNominalOutputReverse(0, 10)
        self.configPeakOutputForward(1, 10)
        self.configPeakOutputReverse(-1, 10)

        self.configVelocityMeasurementPeriod(ctre.VelocityMeasPeriod(1), 10)
        #/* set closed loop gains in slot0 */
        self.config_kF(0, self.pid['kF'], 10)
        self.config_kP(0, self.pid['kP'], 10)
        self.config_kI(0, self.pid['kI'], 10)
        self.config_kD(0, self.pid['kD'], 10)

    def set(self, speed):
        if self.pid != None:
            return ctre.WPI_TalonSRX.set(self, self.controlType, speed * self.kPreScale)
        else:
            return self.set(speed)

class WPI_TalonFXFeedback(ctre.WPI_TalonFX):
    def __init__(self, motorDescription):
        ctre.WPI_TalonFX.__init__(self,motorDescription['channel'])
        self.motorDescription = motorDescription
        self.pid = None

    def setupPid(self,motorDescription = None):
        if not motorDescription:
            motorDescription = self.motorDescription
        if not 'pid' in self.motorDescription:
            print("Motor channel %d has no PID"%(self.motorDescription['channel']))
            return
        self.pid = self.motorDescription['pid']
        self.controlType = self.pid['controlType']
        if self.controlType == "Position":
            self.controlType = ctre.ControlMode.Position
        elif self.controlType == "Velocity":
            self.controlType = ctre.ControlMode.Velocity
        
        self.configSelectedFeedbackSensor(ctre.FeedbackDevice(self.pid['feedbackDevice']), 0, 10)
        self.setSensorPhase(self.pid['sensorPhase'])
        self.pidControlType = self.pid['controlType']
        self.kPreScale = self.pid['kPreScale']

        #/* set the peak, nominal outputs, and deadband */
        self.configNominalOutputForward(0, 10)
        self.configNominalOutputReverse(0, 10)
        self.configPeakOutputForward(1, 10)
        self.configPeakOutputReverse(-1, 10)

        self.configVelocityMeasurementPeriod(ctre.VelocityMeasPeriod(1), 10)
        #/* set closed loop gains in slot0 */
        self.config_kF(0, self.pid['kF'], 10)
        self.config_kP(0, self.pid['kP'], 10)
        self.config_kI(0, self.pid['kI'], 10)
        self.config_kD(0, self.pid['kD'], 10)

    def set(self, speed):
        if self.pid != None:
            return ctre.WPI_TalonFX.set(self, self.controlType, speed * self.kPreScale)
        else:
            return self.set(speed)

class SparkMaxFeedback(rev.CANSparkMax):
    def __init__(self, motorDescription, motors):
        self.motorDescription = motorDescription
        rev.CANSparkMax.__init__(self, self.motorDescription['channel'], self.motorDescription['motorType'])
        self.setInverted(self.motorDescription['inverted'])
        self.motors = motors

    def setupPid(self):
        '''Sets up the PIDF values and a pidcontroller to use to control the motor using pid.'''
        if not 'pid' in self.motorDescription:
            print("Motor channel %f has no PID", (self.motorDescription['channel']))
            return
        self.pid = self.motorDescription['pid']
        pid = self.pid
        self.ControlType = pid['controlType']
        if self.ControlType == "Position":
            self.ControlType = rev.ControlType.kPosition
        elif self.ControlType == "Velocity":
            self.ControlType == rev.ControlType.kVelocity
        self.encoder = self.getEncoder()

        self.kPreScale = pid['kPreScale']
        self.PIDController = self.getPIDController() #creates pid controller

        self.PIDController.setP(pid['kP'], pid['feedbackDevice'])
        self.PIDController.setI(pid['kI'], pid['feedbackDevice'])
        self.PIDController.setD(pid['kD'], pid['feedbackDevice'])
        self.PIDController.setFF(pid['kF'], pid['feedbackDevice'])

        self.PIDController.setOutputRange(-1, 1, pid['feedbackDevice'])
        self.PIDController.setReference(0 , self.pidControlType, pid['feedbackDevice']) #Sets the control type to velocity on the pid slot we passed in

    def setControlType(self, type):
        '''Use the rev.ControlType.k(control type) as arg'''
        if self.ControlType == "Position":
            self.ControlType = rev.ControlType.kPosition
        elif self.ControlType == "Velocity":
            self.ControlType == rev.ControlType.kVelocity
        else:
            log.debug("Wrong type. Use either Position or Velocity")

    def set(self, speed):
        if self.motorDescription['type'] != "SparkMaxFollower":
            log.debug("error = %f", (speed*self.pid['kPreScale'])-self.encoder.getVelocity())
            return self.PIDController.setReference(speed*self.pid['kPreScale'], self.ControlType)
        return
