

class intakePivot:
    def __init__(self, ):
        #Constants
        #Will most likely change in the future
        #seperate file
        self.kMinSpeed: float = 0
        self.kMaxSpeed: float = 90
        self.kMinPos: float = 0
        self.kMaxPos: float = 135
        
        self.limitSwitchUpper = limitSwitch(0?)
        self.limitSwitchUpper = limitSwitch(1?)

        self.IntakePivotMotor = motor(20?)

        self.IntakePivorMotorEncoder = motorEncoder(20?)

        self.setGoal(0)

    def setGoal(self, goal: float) -> None:
        self.goal = goal
    
    def distCalc(self) -> None:
        self.diff = self.IntakePivorMotorEncoder - self.goal

    def setMotorSpeed(self, motorSpeed: float) -> None:
        self.motorSpeed = motorSpeed

    def checkMotorSpeed(self) -> None:
        if self.diff <= 27 #degrees, assuming 90 max speed & 150 max acceleration
            self.setMotorSpeed(0.1)
        
    #add math for degrees before slow down

    def hardConstraints(self) -> None:
        if self.limitSwitchUpper or self.limitSwitchUpper:
            self.setMotorSpeed(0)
        if self.IntakePivorMotorEncoder < self.kMinPos or self.IntakePivorMotorEncoder > self.kMaxPos:
            self.setMotorSpeed(0)
    
    def turnToGoal(self) -> None:
        

    def periodic(self) -> None:
        self.checkMotorSpeed()
        self.checkHardConstraints()
        self.distCalc()
