import wpilib
import rev
import commands2

from constants import CaptainPlanetConsts as consts, DiverCarlChute as chute_consts


class CaptainIntake(commands2.Subsystem):

    def __init__(self) -> None:
        super().__init__()
        self.intakeMotor = rev.SparkMax(
            consts.kMotorCanId, rev.SparkLowLevel.MotorType.kBrushless
        )
        self.chuteMotor = rev.SparkMax(
            chute_consts.kMotorCanId, rev.SparkLowLevel.MotorType.kBrushless
        )
        # Front is Large Green Wheel
        self.front_breakbeam = wpilib.DigitalInput(consts.kFrontBreakBeam)
        self.back_breakbeam = wpilib.DigitalInput(consts.kBackBreakBeam)
        self.smartdashboard = wpilib.SmartDashboard
        self.intake_in_progress = False
        self.ready_to_eject = False

    def setMotor(self, speed: float):

        self.intakeMotor.set(speed)
        self.chuteMotor.set(speed)

    def updateDashboard(self, state: str):
        # Should be called in periodic
        # Should also be run
        self.smartdashboard.putBoolean("Intake/Front 1", self.front_breakbeam.get())
        self.smartdashboard.putBoolean("Intake/Back 0", self.back_breakbeam.get())
        self.smartdashboard.putBoolean("Intake/In Progress", self.intake_in_progress)
        self.smartdashboard.putString("Intake/State", state)

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

    def initialize(self) -> None:
        self.intake.setMotor(0)
        self.intake_in_progress = False

    def execute(self) -> None:
        # Idle speed command initialze with idle speed from Rayl
        # self.intake.idleSpeed from captn intake
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
        return not self.intake.front_breakbeam.get()

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
        return not self.intake.back_breakbeam.get()

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
            self.intake.front_breakbeam.get() and not self.intake.back_breakbeam.get()
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
        if self.intake.front_breakbeam.get():
            self.intake.ready_to_eject = True
            return True
        return False


class ReadyToEject(commands2.Command):
    """We're in this state if we've properly taken a piece in and are ready to eject it"""

    def __init__(self, intake: CaptainIntake) -> None:
        super().__init__()
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self) -> None:
        self.intake.updateDashboard("readytoeject")

    def isFinished(self) -> bool:
        """When the button is pressed (again) eject

        Returns:
            bool: _description_
        """
        # If we're not ready to eject (becasue we didn't correctly load, immediately abort)
        return not self.intake.ready_to_eject or self.intake.smartdashboard.getBoolean(
            "A Button Pressed", False
        )


class Eject(commands2.ParallelRaceGroup):
    """Eject the piece"""

    def __init__(self, intake: CaptainIntake) -> None:
        super().__init__(
            commands2.RunCommand(lambda: intake.setMotor(consts.kDefaultSpeed), intake),
            commands2.WaitCommand(consts.kEjectTime),
        )
        self.intake = intake
        self.addRequirements(intake)

    def initialize(self) -> None:
        self.intake.updateDashboard("eject")

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool):
        self.intake.setMotor(0)
        self.intake.ready_to_eject = False


class CaptainIntakeStateMachine(commands2.SequentialCommandGroup):

    def __init__(self, intake: CaptainIntake):
        super().__init__()

        self.addCommands(
            commands2.RepeatCommand(
                commands2.SequentialCommandGroup(
                    IdleState(intake),
                    # Wrap the sequence in a ParallelRaceGroup with a timeout
                    commands2.ParallelRaceGroup(
                        commands2.SequentialCommandGroup(
                            FirstIntaking(intake),
                            SecondIntaking(intake),
                            ThirdIntaking(intake),
                            Intook(intake),
                            BackItUp(intake),
                        ),
                        # timeout will make the state machine go back to idle
                        commands2.WaitCommand(consts.kIntakeTimeout),
                    ),
                    ReadyToEject(intake),
                    Eject(intake),
                )
            )
        )
