from robotpy_ext.autonomous import StatefulAutonomous, state

class StreamDeck(StatefulAutonomous):
    MODE_NAME = "StreamDeck"

    def __init__(self):
        super().__init__()

    @state(first=True)
    def idle(self):
        self.next_state("idle")

