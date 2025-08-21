from subsystems.intakeSubsystems import intakeMotors, intakePivot

class intakeSubsystem():
	def __init__(self, ):
		try:
			self.intakemotors = intakemotors()
			self.intakepivot = intakepivot()
		exept Exeption:
			sys.exit("Either intakeMotors and/or intakePivot is not seen by intakeSubsystem")
	execute
		def runMotors(self, spitOut: bool):
			self.intakemotors.runMotor(spitOut)

		def setGoal(self, goal: float):
			self.intakePivot.setGoal(goal)
		
		def turnToGoal(self):
			self.intakepivot.turnToGoal()
