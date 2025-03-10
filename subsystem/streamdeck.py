import wpilib
import commands2
import ntcore

from robotpy_ext.autonomous import StatefulAutonomous, state

class StreamDeck(StatefulAutonomous):
    MODE_NAME = "StreamDeck"

    def __init__(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()

        self.table = self.inst.getTable("Stream_Deck")

        self.table.putNumber("pressedKey", -1)
        
        self.smartdashboard = wpilib.SmartDashboard()
        super().__init__()

    @state(first=True)
    def idle(self):
        self.keyPressed = self.table.getNumber("pressedKey", -1)
        self.heartbeat = self.table.getNumber("Stream Deck Heartbeat", 0)
        wpilib.SmartDashboard.putNumber("Stream Deck Life", self.heartbeat) # TODO Need to make something seen on the streamdeck or smartdashboard

        match self.keyPressed:
            case 0:
                self.next_state("reef_position_L4")
            case 1:
                self.next_state("reef_position_R4")
            case 2:
                self.next_state("remove_algae_upper")
            case 3:
                self.next_state("idle")
            case 4:
                self.next_state("net_algae")
            case 5:
                self.next_state("reef_position_L3")
            case 6:
                self.next_state("reef_position_R3")
            case 7:
                self.next_state("remove_algae_lower")
            case 8:
                self.next_state("idle")
            case 9:
                self.next_state("eject_algae")
            case 10:
                self.next_state("reef_position_L2")
            case 11:
                self.next_state("reef_position_R2")
            case 12:
                self.next_state("trough")
            case 13:
                self.next_state("coral_chute_intake")
            case 14:
                self.next_state("eject_coral")
            case -1:
                self.next_state("idle")

        # self.next_state("idle")

    @state()
    def reef_position_L4(self):
        self.smartdashboard.putString("AutoAlign.Position", "L4")
        self.smartdashboard.putString("Pivot.Position", "L4")
        self.smartdashboard.putString("Elevator.Position", "L4")
    
    @state()
    def reef_position_L3(self):
        self.smartdashboard.putNumber("AutoAlign.Position", "L3")
        self.smartdashboard.putString("Pivot.Position", "L3")
        self.smartdashboard.putString("Elevator.Position", "L3")

    @state()
    def reef_position_L2(self):
        self.smartdashboard.putNumber("AutoAlign.Position", "L2")
        self.smartdashboard.putString("Pivot.Position", "L2")
        self.smartdashboard.putString("Elevator.Position", "L2")

    @state()
    def reef_position_R4(self):
        self.smartdashboard.putNumber("AutoAlign.Position", "R4")
        self.smartdashboard.putString("Pivot.Position", "R4")
        self.smartdashboard.putString("Elevator.Position", "R4")

    @state()
    def reef_position_R3(self):
        self.smartdashboard.putNumber("AutoAlign.Position", "R3")
        self.smartdashboard.putString("Pivot.Position", "R3")
        self.smartdashboard.putString("Elevator.Position", "R3")

    @state()
    def reef_position_R2(self):
        self.smartdashboard.putNumber("AutoAlign.Position", "R2")
        self.smartdashboard.putString("Pivot.Position", "R2")
        self.smartdashboard.putString("Elevator.Position", "R2")

    @state()
    def remove_algae_upper(self):
        self.smartdashboard.putNumber("AutoAlign.Position", "L3_algae")
        self.smartdashboard.putString("Pivot.Position", "L3_algae")
        self.smartdashboard.putString("Elevator.Position", "L3_algae")
        # run intake & pivot to get the algae out

    @state()
    def remove_algae_lower(self):
        self.smartdashboard.putNumber("AutoAlign.Position", "L2_algae")
        self.smartdashboard.putString("Pivot.Position", "L2_algae")
        self.smartdashboard.putString("Elevator.Position", "L2_algae")
        # run intake & pivot to get the algae out

    @state()
    def net_algae(self):
        self.smartdashboard.putString("Pivot.Position", "net")
        self.smartdashboard.putString("Elevator.Position", "net")
        # intake unintakes the algae

    @state()
    def eject_algae(self):
        self.smartdashboard.putString("Pivot.Position", "eject_algae")
        self.smartdashboard.putString("Elevatot.Position", "eject_algae")
        # intake unintakes the algae

    @state()
    def trough(self):
        self.smartdashboard.putNumber("AutoAlign.Position", "trough")
        self.smartdashboard.putString("Pivot.Position", "trough")
        self.smartdashboard.putString("Elevator.Position", "trough")

    @state()
    def eject_coral(self):
        self.smartdashboard.putString("Pivot.Position", "L2_algae")
        self.smartdashboard.putString("Elevator.Position", "L2_algae")
        # unintake algae

    @state()
    def coral_chute_intake(self):
        self.smartdashboard.putNumber("AutoAlign.Position", "chute_intake")
        self.smartdashboard.putString("Pivot.Position", "chute_intake")
        self.smartdashboard.putString("Elevator.Position", "chute_intake")
