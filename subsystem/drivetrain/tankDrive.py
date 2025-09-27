import commands2
import wpilib
import wpilib.drive


class TankDrive(commands2.SubsystemBase):
    def __init__(self, left_motors, right_motors):
        """
        Set up the drivetrain with two collections of motors, one for each side
        of the robot, and pass them into a differential drive interface.

        Args:
            left_motors: motor controller group containing motors for the left
                side of the drivetrain
            right_motors: motor controller group containing motors for the right
                side of the drivetrain

        Returns:
            None
        """
        # Store motors within the object
        self.left_motors = left_motors
        self.right_motors = right_motors

        # Invert the rotation direction on one side of the drivetrain, because
        # the motors on each side face opposite from one another
        self.right_motors.setInverted(True)

        # Enable differential drive functionality within the subsystem
        self.drive_train = wpilib.drive.DifferentialDrive(self.left_motors, self.right_motors)

    def setDeadband(self, deadband):
        self.drive_train.setDeadband(deadband)

    def tankDrive(self, left_input_perc, right_input_perc, square_inputs=True):
        """
        Operate the drivetrain using tank drive, meaning that the speeds for
        each side of the drivetrain are explicitly specified using human input.

        Args:
            left_input_perc: float with domain [-1, 1] setting percentage of
                output to run the left drive motors at
            right_input_perc: float with domain [-1, 1] setting percentage of
                output to run the right drive motors at
            square_inputs: True if the two input percentages should be squared
                before passing to the drivetrain, False otherwise. Defaults to True

        Returns:
            None
        """
        # Operate the robot using tank drive
        self.drive_train.tankDrive(
            left_input_perc, right_input_perc, squareInputs=square_inputs
        )