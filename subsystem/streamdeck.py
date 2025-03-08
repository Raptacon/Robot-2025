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
        wpilib.SmartDashboard.putNumber("Stream Deck Life", self.heartbeat)

        match self.keyPressed:
            case 0:
                self.next_state("align_to_reef_position_L4")
            case 1:
                self.next_state("align_to_reef_position_R4")
            case 3:
                self.next_state("remove_algae_upper")

        # self.next_state("idle")

    @state()
    def align_to_reef_position_L4(self):
        
    
    @state()
    def align_to_reef_position_L3(self):
        pass

    @state()
    def align_to_reef_position_L2(self):
        pass

    @state()
    def align_to_reef_position_R4(self):
        pass

    @state()
    def align_to_reef_position_R3(self):
        pass

    @state()
    def align_to_reef_position_R2(self):
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
    def align_to_trough(self):
        pass

    @state()
    def eject_coral(self):
        pass

    @state()
    def coral_chute_intake(self):
        pass
