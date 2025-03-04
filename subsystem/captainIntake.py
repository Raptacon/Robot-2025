import wpilib
import rev

from robotpy_ext.autonomous import StatefulAutonomous, state
from constants import CaptainPlanetConsts as consts


class CaptainIntake(StatefulAutonomous):
    MODE_NAME = "Intake"

    def __init__(self):
        self.intakeMotor = rev.SparkMax(consts.kMotorCanId, rev.SparkLowLevel.MotorType.kBrushless)
        self.front_breakbeam = wpilib.DigitalInput(consts.kFrontBreakBeam)
        self.back_breakbeam = wpilib.DigitalInput(consts.kBackBreakBeam)
        self.smartdashboard = wpilib.SmartDashboard
        super().__init__()

    @state(first=True)
    def idle(self):
        self.intakeMotor.set(0)
        
        self.smartdashboard.putBoolean("Breakbeam 1", self.front_breakbeam.get())
        self.smartdashboard.putBoolean("Breakbeam 2", self.back_breakbeam.get())
        self.smartdashboard.putString("Intake State", "idle")

        if self.smartdashboard.getBoolean("A Button Pressed", False):
            self.next_state("first_intaking")

        if not self.smartdashboard.getBoolean("A Button Pressed", False):
            self.next_state("idle")

    @state()
    def first_intaking(self):        
        self.intakeMotor.set(consts.kDefaultSpeed)

        self.smartdashboard.putBoolean("Breakbeam 1", self.front_breakbeam.get())
        self.smartdashboard.putBoolean("Breakbeam 2", self.back_breakbeam.get())
        self.smartdashboard.putString("Intake State", "first_intaking")

        

        if not self.front_breakbeam.get():
            self.next_state("second_intaking")
        if not self.smartdashboard.getBoolean("A Button Pressed", False):
            self.next_state("idle") 

    @state()
    def second_intaking(self):        
        self.intakeMotor.set(consts.kDefaultSpeed)

        self.smartdashboard.putBoolean("Breakbeam 1", self.front_breakbeam.get())
        self.smartdashboard.putBoolean("Breakbeam 2", self.back_breakbeam.get())
        self.smartdashboard.putString("Intake State", "second_intaking")

        

        if not self.back_breakbeam.get():
            self.next_state("third_intaking")

    @state()
    def third_intaking(self):        
        self.intakeMotor.set(consts.kDefaultSpeed)

        self.smartdashboard.putBoolean("Breakbeam 1", self.front_breakbeam.get())
        self.smartdashboard.putBoolean("Breakbeam 2", self.back_breakbeam.get())
        self.smartdashboard.putString("Intake State", "third_intaking")

        

        if self.front_breakbeam and not self.back_breakbeam.get():
            self.next_state("intook")

    @state()
    def intook(self):
        self.intakeMotor.set(0)

        self.smartdashboard.putBoolean("Breakbeam 1", self.front_breakbeam.get())
        self.smartdashboard.putBoolean("Breakbeam 2", self.back_breakbeam.get())
        self.smartdashboard.putString("Intake State", "intook")

        self.next_state("idle")