# Native imports
import math
from typing import Callable

# Internal imports
from constants import SwerveDriveConsts
from subsystem.drivetrain.drivetrain import SwerveDrivetrain

# Third-party imports
import commands2

class DefaultDrive(commands2.Command):
    """
    """
    def __init__ (
        self,
        driveTrain: SwerveDrivetrain,
        velocity_vector_x: Callable[[], float],
        velocity_vector_y: Callable[[], float],
        angular_velocity: Callable[[], float],
        field: Callable[[], bool]
    ) -> None:
        """
        """
        super().__init__()

        self.driveTrain = driveTrain
        self.velocity_vector_x = velocity_vector_x
        self.velocity_vector_y = velocity_vector_y
        self.angular_velocity = angular_velocity
        self.field = field
        self.addRequirements(self.driveTrain)

    def execute(self) -> None:
        """
        """
        self.driveTrain.drive(
            self.velocity_vector_x() * SwerveDriveConsts.maxTranslationMPS,
            self.velocity_vector_y() * SwerveDriveConsts.maxTranslationMPS,
            self.angular_velocity() * math.radians(SwerveDriveConsts.maxAngularDPS),
            # TODO: JD make button configured
            self.field
        )
