import wpilib
import rev

from robotpy_ext.autonomous import StatefulAutonomous, state
from constants import CaptainPlanetConsts as consts


class CaptainIntake(StatefulAutonomous):
    MODE_NAME = "Intake"

    def __init__(self):
        self.intakeMotor = rev.SparkMax(consts.kMotorCanId, rev.SparkLowLevel.MotorType.kBrushless)
        self.breakbeam_1 = wpilib.DigitalInput(consts.kFrontBreakBeam)
        self.breakbeam_2 = wpilib.DigitalInput(consts.kBackBreakBeam)
        self.smartdashboard = wpilib.SmartDashboard
        super().__init__()

    @state(first=True)
    def idle(self):
        self.smartdashboard.putBoolean("Breakbeam 1", not self.breakbeam_1.get())
        self.smartdashboard.putBoolean("Breakbeam 2", not self.breakbeam_2.get())

        if self.smartdashboard.getBoolean("A Button Pressed", False):
            self.next_state("intaking")

    @state()
    def intaking(self):        
        self.intakeMotor.set(consts.kDefaultSpeed)

        if self.breakbeam_1 and not self.breakbeam_2:
            self.next_state("intook")

    @state()
    def intook(self):
        self.intakeMotor.set(0)

        self.next_state("idle")