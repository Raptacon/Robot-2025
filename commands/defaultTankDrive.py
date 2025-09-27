import commands2


class DefaultTankDrive(commands2.Command):
    """
    Operate a differential drive robot using tank drive. This involves
    specifying the speed of each side directly through the use of joysticks.
    """
    def __init__(self, left_input_percentage, right_input_percentage, drivetrain):
        """
        Set up the command with two input percentages, one for each side of the
        robot, along with the required drivetrain subsystem.

        Args:
            left_input_percentage: callable returning the motor output percentage
                (with domain [-1, 1]) to apply to the left side
            right_input_percentage: callable returning the motor output percentage
                (with domain [-1, 1]) to apply to the right side
            drivetrain: subsystem representing all of the drive motors

        Returns:
            None
        """
        # Easy access to functionality from commands2.Command
        super().__init__()

        # Store arguments as instance attributes for use in methods
        self.left_input_percentage = left_input_percentage
        self.right_input_percentage = right_input_percentage
        self.drivetrain = drivetrain

        # Tell the robot executor that this command requires the drivetrain
        # subsystem
        self.addRequirements(self.drivetrain)

    def execute(self):
        """
        Operate the robot using tank drive based on left and right input
        percentages from joysticks. Executes once every clock tick.

        Returns:
            None
        """
        self.drivetrain.tankDrive(self.left_input_percentage(), self.right_input_percentage())
    
    def end(self, interrupted):
        """
        Shut down motor output on both sides of the drivetrain whne the command
        terminates.

        Args:
            interrupted: True if the command ends because it was interrupted
        
        Returns:
            None
        """
        self.drivetrain.tankDrive(0, 0)
    
    def isFinished(self):
        """
        Define when the command is considered complete. Because the drivetrain
        is always operated by the driver using tank drive, this command is
        never considered complete and we return False.

        Returns:
            Boolean representing whether command is complete
        """
        return False
    