import commands2
import typing
from subsystem.drivetrain.swerve_drivetrain import SwerveDrivetrain

class ToggleFieldDrive(commands2.InstantCommand):
    def __init__(self,
                 driveTrain: SwerveDrivetrain,
                 robotRelative: typing.Callable[[],bool]) -> None:
        super().__init__()
        self.driveTrain = driveTrain
        self.robotRelative = robotRelative
        self.addRequirements(driveTrain)

    def execute(self) -> None:
        print(f"Toggling field relative from {self.driveTrain.getFieldDriveRelative()} to {not self.robotRelative()}")
        self.driveTrain.setFieldDriveRelative(not self.robotRelative())
