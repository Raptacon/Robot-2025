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
        self.keys = {0: commands2.cmd.print_("Key 0 pressed"),
                     1: commands2.cmd.print_("Key 1 pressed"),
                     2: commands2.cmd.print_("Key 2 pressed"),
                     3: commands2.cmd.print_("Key 3 pressed"),
                     4: commands2.cmd.print_("Key 4 pressed"),
                     5: commands2.cmd.print_("Key 5 pressed"),
                     6: commands2.cmd.print_("Key 6 pressed"),
                     7: commands2.cmd.print_("Key 7 pressed"),
                     8: commands2.cmd.print_("Key 8 pressed"),
                     9: commands2.cmd.print_("Key 9 pressed"),
                     10: commands2.cmd.print_("Key 10 pressed"),
                     11: commands2.cmd.print_("Key 11 pressed"),
                     12: commands2.cmd.print_("Key 12 pressed"),
                     13: commands2.cmd.print_("Key 13 pressed"),
                     14: commands2.cmd.print_("Key 14 pressed"),
                     -1: commands2.cmd.print_("No key pressed"),}
        self.smartdashboard = wpilib.SmartDashboard()
        super().__init__()

    @state(first=True)
    def idle(self):
        self.keyPressed = self.table.getNumber("pressedKey", -1)
        self.heartbeat = self.table.getNumber("Stream Deck Heartbeat", 0)
        wpilib.SmartDashboard.putNumber("Stream Deck Life", self.heartbeat)
        self.next_state("idle")

    @state()
    def align_to_reef_position(self):
        self.smartdashboard.put("Auto Align", )