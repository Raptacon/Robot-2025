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
        self.smartdashboard.putString("Chute.Position", "L4")
        self.smartdashboard.putString("Elevator.Position", "L4")
    
    @state()
    def reef_position_L3(self):
        self.smartdashboard.putNumber("L3 Reef Position", self.keyPressed)

    @state()
    def reef_position_L2(self):
        pass

    @state()
    def reef_position_R4(self):
        pass

    @state()
    def reef_position_R3(self):
        pass

    @state()
    def reef_position_R2(self):
        pass

    @state()
    def remove_algae_upper(self):
        pass

    @state()
    def remove_algae_lower(self):
        pass

    @state()
    def net_algae(self):
        pass

    @state()
    def eject_algae(self):
        pass

    @state()
    def trough(self):
        pass

    @state()
    def eject_coral(self):
        pass

    @state()
    def coral_chute_intake(self):
        pass
