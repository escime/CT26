from commands2 import Subsystem
from constants import LEDConstants
from wpilib import PWM


class LEDSubsystem(Subsystem):
    def __init__(self):
        super().__init__()

        self.state_constants = {
            "default": 1100,
            "purple_flashing": 1200,
            "rainbow_chaser" : 1300,
            "yellow_chaser": 1400,
            "yellow_red_chaser": 1500,
            "yellow_blue_chaser": 1600,
            "white_flashing": 1700,
            "unused_1": 1800,
            "unused_2": 1900,
        }
        self.pwm = PWM(LEDConstants.port)

        self.state = "default"
        self.last_state = "default"

        self.pwm.setPulseTime(self.state_constants[self.state])

    def set_state(self, target_state: str) -> None:
        """Set the current state of the subsystem."""
        self.state = target_state

    def get_state(self) -> str:
        return self.state

    def periodic(self) -> None:
        if self.last_state != self.state:
            self.pwm.setPulseTime(self.state_constants[self.state])
            self.last_state = self.state
