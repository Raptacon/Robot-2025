from Commands import commands2
from subsystems import intakePivot, intakeMotors, indexer

indexerVar = indexer()

#intake commands
intakeMotorVar = intakeMotor()
intakePivotVar = intakePivot()

#intake motor commands
intakingMotor = commands2.cmd.run(lambda: intakeMotorVar.runMotor(False), intakeMotorVar)
outtakingMotor = commands2.cmd.run(lambda: intakeMotorVar.runMotor(True), intakeMotorVar).withTimeout(3)
stoppingMotor = commands2.cmd.run(intakeMotorVar.stopIntakeMotor, intakeMotorVar)

#intake pivot commands
turnToDeploy = commands2.cmd.runEnd(lambda: intakePivotVar.setGoal(0), intakePivotVar.turnToGoal, intakePivotVar)
turnToHandOff = commands2.cmd.runEnd(lambda: intakePivotVar.setGoal(135), intakePivotVar.turnToGoal, intakePivotVar)
stopTurn = commands2.cmd.run(intakePivotVar.stopPivotMotor, intakePivotVar)

#intake combined commands
ingestUntilFull = commands2.cmd.runEnd(
    lambda: intakePivotVar.setGoal(135), commands2.cmd.parallel(
        intakePivotVar.turnToGoal, lambda: intakeMotorVar.runMotor(False)
    ), intakePivotVar, intakeMotorVar
)
upAndOff = commands2.cmd.sequence(intakeMotorVar.stopIntakeMotor, lambda: intakePivotVar.setGoal(135), intakePivotVar.turnToGoal)
downAndOn = commands2.cmd.sequence(lambda: intakePivotVar.setGoal(0), intakePivotVar.turnToGoal, lambda: intakeMotorVar.runMotor(False))
stopAllIntake = commands2.cmd.runEnd(intakeMotorVar.stopIntakeMotor, intakePivotVar.stopPivotMotor, intakeMotorVar, intakePivotVar)

class intakeCommands():
    intakeMotorVar = intakeMotor()
    intakePivotVar = intakePivot()

    #intake motor commands
    def intakeingMotorCommand(self):
        intakingMotor = commands2.cmd.run(lambda: intakeMotorVar.runMotor(False), intakeMotorVar)
        return intakingMotor

    def outtakningMotorCommand(self):
        outtakingMotor = commands2.cmd.run(lambda: intakeMotorVar.runMotor(True), intakeMotorVar).withTimeout(3)
        return outtakingMotor

    def stopIntakeMotorCommand(self):
        stoppingMotor = commands2.cmd.run(intakeMotorVar.stopIntakeMotor, intakeMotorVar)
        return stoppingMotor
    
    #intake pivot commands
    def turnToDeployCommand(self):
        turnToDeploy = commands2.cmd.runEnd(lambda: intakePivotVar.setGoal(0), intakePivotVar.turnToGoal, intakePivotVar)
        return turnToDeploy

    def turnToHandOffCommand(self):
        turnToHandOff = commands2.cmd.runEnd(lambda: intakePivotVar.setGoal(135), intakePivotVar.turnToGoal, intakePivotVar)
        return turnToHandOff

    def stopTurnCommand(self):
        stopTurn = commands2.cmd.run(intakePivotVar.stopPivotMotor, intakePivotVar)
        return stopTurn

    #combined intake commmands
    def ingestUntilFullCommand(self):
        ingestUntilFull = commands2.cmd.deadline(
            self.indexerVar.checkIndexerSensor(), 
        )
        return ingestUntilFull

    def upAndOffCommand(self):
        # turn into parrallel
        upAndOff = commands2.cmd.sequence(intakeMotorVar.stopIntakeMotor, lambda: intakePivotVar.setGoal(135), intakePivotVar.turnToGoal)
        return upAndOff

    def downAndOnCommand(self):
        # turn into parrallel
        downAndOn = commands2.cmd.sequence(
            lambda: intakePivotVar.setGoal(0), intakePivotVar.turnToGoal, lambda: intakeMotorVar.runMotor(False)
        )
        return downAndOn

    def stopAllIntakeCommand(self):
        stopAllIntake = commands2.cmd.parellel(
            commands2.cmd.run(intakeMotorVar.stopIntakeMotor, intakeMotorVar), 
            commands2.cmd.run(intakePivotVar.stopPivotMotor, intakePivotVar)
        )
        return stopAllIntake
    