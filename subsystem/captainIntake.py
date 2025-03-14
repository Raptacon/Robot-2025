import wpilib
import rev
import commands2
from typing import Callable

from constants import CaptainPlanetConsts as consts, DiverCarlChute as chute_consts


class CaptainIntake(commands2.Subsystem):

    def __init__(self) -> None:
        super().__init__()
        self.intakeMotor = rev.SparkFlex(
            consts.kMotorCanId, rev.SparkLowLevel.MotorType.kBrushless
        )
        self.chuteMotor = rev.SparkFlex(
            chute_consts.kMotorCanId, rev.SparkLowLevel.MotorType.kBrushless
        )
        # Front is Large Green Wheel
        self.front_breakbeam = wpilib.DigitalInput(consts.kFrontBreakBeam)
        self.back_breakbeam = wpilib.DigitalInput(consts.kBackBreakBeam)
        self.smartdashboard = wpilib.SmartDashboard
        self.idleSpeed = 0

    def setMotor(self, speed: float):
        self.intakeMotor.set(speed)
        self.chuteMotor.set(speed)

    def updateDashboard(self, state: str):
        self.smartdashboard.putBoolean("Front 1", self.front_breakbeam.get())
        self.smartdashboard.putBoolean("Back 2", self.back_breakbeam.get())
        self.smartdashboard.putString("Intake State", state)

    def getBreakBeam(self, position: consts.BreakBeam) -> bool:
        if position == consts.BreakBeam.FRONT:
            return self.front_breakbeam.get()
        else:
            return self.back_breakbeam.get()


# FIRST STATE
class IdleState(commands2.Command):

    def __init__(self, intake: CaptainIntake) -> None:
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)

    def execute(self) -> None:
        self.intake.setMotor(self.intake.idleSpeed)
        self.intake.updateDashboard("idle")

    def isFinished(self) -> bool:
        return self.intake.smartdashboard.getBoolean("A Button Pressed", False)

class FirstIntaking(commands2.Command):

    def __init__(self, intake: CaptainIntake) -> None:
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self) -> None:
        self.intake.setMotor(consts.kDefaultSpeed)

    def execute(self) -> None:
        self.intake.updateDashboard("first_intaking")

    def isFinished(self) -> bool:
        return (
            not self.intake.front_breakbeam.get()
            or not self.intake.smartdashboard.getBoolean("A Button Pressed", False)
        )

class SecondIntaking(commands2.Command):

    def __init__(self, intake: CaptainIntake) -> None:
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self) -> None:
        self.intake.setMotor(consts.kDefaultSpeed)

    def execute(self) -> None:
        self.intake.updateDashboard("second_intaking")

    def isFinished(self) -> bool:
        return (
            not self.intake.back_breakbeam.get()
            or not self.intake.smartdashboard.getBoolean("A Button Pressed", False)
        )

class ThirdIntaking(commands2.Command):

    def __init__(self, intake: CaptainIntake) -> None:
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self) -> None:
        self.intake.setMotor(consts.kDefaultSpeed)

    def execute(self) -> None:
        self.intake.updateDashboard("third_intaking")

    def isFinished(self) -> bool:
        return (
            self.intake.front_breakbeam.get()
            and not self.intake.back_breakbeam.get()
            or not self.intake.smartdashboard.getBoolean("A Button Pressed", False)
        )

class Intook(commands2.Command):

    def __init__(self, intake: CaptainIntake) -> None:
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self) -> None:
        self.intake.setMotor(0)
        self.intake.updateDashboard("intook")

    def isFinished(self) -> bool:
        return True


class BackItUp(commands2.Command):
    """Backs the piece back up after it has been intook/intaken up the the front breakbeam"""

    def __init__(self, intake: CaptainIntake) -> None:
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self) -> None:
        self.intake.updateDashboard("backitup")
        self.intake.setMotor(-consts.kDefaultSpeed)

    def isFinished(self) -> bool:
        """When the piece is pulled back in the breakbeam stop

        Returns:
            bool: _description_
        """
        return (
            self.intake.front_breakbeam.get()
            or not self.intake.smartdashboard.getBoolean("A Button Pressed", False)
        )


class CaptainIntakeStateMachine(commands2.SequentialCommandGroup):

    def __init__(self, intake: CaptainIntake):
        super().__init__()

        self.addCommands(
            commands2.RepeatCommand(
                commands2.SequentialCommandGroup(
                    IdleState(intake),
                    # If A button is released, go back to idle
                    commands2.ConditionalCommand(
                        # If A button still pressed, continue sequence
                        commands2.SequentialCommandGroup(
                            FirstIntaking(intake),
                            SecondIntaking(intake),
                            ThirdIntaking(intake),
                            Intook(intake),
                            BackItUp(intake),
                        ),
                        # If A button released, go back to idle
                        commands2.InstantCommand(),
                        # Condition: A button still pressed
                        lambda: intake.smartdashboard.getBoolean(
                            "A Button Pressed", False
                        ),
                    ),
                )
            )
        )

class SetCaptainIntakeIdleSpeed(commands2.Command):
    def __init__(self,
                 intake: CaptainIntake,
                 getIdleSpeed: Callable[[], float]):
        self.intake = intake
        self.getIdleSpeed = getIdleSpeed
        self.addRequirements(self.intake)
    
    def execute(self):
        self.intake.idleSpeed = self.getIdleSpeed()
    
    def end(self):
        self.intake.idleSpeed = 0
