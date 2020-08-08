from networktables import NetworkTables as networktable
from magicbot import StateMachine, tunable
from magicbot.state_machine import state, timed_state
from components.driveTrain import DriveTrain
from components.shooterLogic import ShooterLogic



class AutoAim(StateMachine):
    compatString = ["doof"]
    time = 0.1
    driveTrain: DriveTrain
    shooter: ShooterLogic
    drive_speed_left = tunable(.05)
    drive_speed_right = tunable(-.05)
    minAimOffset = .5;

    @state(first = True)
    def start(self):
        table = networktable.getTable("limelight")
        if table.getNumber("tv", -1) == 1: #If limelight has any valid targets
            tx = table.getNumber("tx", -50) # "-50" is the default value, so if that is returned, nothing should be done because there is no connection.
            if tx != -50:
                if tx > minAimOffset:
                    self.next_state_now("adjust_self_left")
                elif tx < -1*minAimOffset:
                    self.next_state_now("adjust_self_right")
                elif tx < minAimOffset and tx > -1*minAimOffset:
                    self.next_state_now("stop_shoot")
                else:
                    self.next_state_now("stop")


    @timed_state(duration = time, next_state = "start")
    def adjust_self_left(self):
        """Drives the bot backwards for a time"""
        self.driveTrain.setTank(self.drive_speed_left, self.drive_speed_right) #We could do this based off of PID and error at some point, instead of timing.
    
    @timed_state(duration = time, next_state = "start")
    def adjust_self_right(self):
        """Drives the bot backwards for a time"""
        self.driveTrain.setTank(self.drive_speed_right, self.drive_speed_left)

    @state(must_finish = True)
    def stop_shoot(self):
        #stop
        self.driveTrain.setTank(0, 0)  
        #shoot
        self.shooter.shootBalls()
