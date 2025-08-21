

class intakeMotors:
    def __init__(self):
        #consts, seperate file later
        #might change to having 2 seperate motor speeds later for intaking and outtaking
        self.kMotorSpeed: float = 0.6

        self.intakeMotor = self.motor(21)

        # self.intakeMotorEncoder = self.motorEncoder(21)
        # possibly use to detect a change in current
        
    def runMotor(self, spitOut: bool)
        if not spitOut:
            self.intakeMotor.set(self.kMotorSpeed)
        else:
            self.intakeMotor.set(-self.kMotorSpeed)

    def stopIntakeMotor(self):
        self.intakeMotor.set(0)
