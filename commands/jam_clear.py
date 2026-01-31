from commands2 import Command

from subsystems.hoppersubsystem import HopperSubsystem
from subsystems.intakesubsystem import IntakeSubsystem

class JamClear(Command):
    def __init__(self, hopper: HopperSubsystem, intake: IntakeSubsystem):
        super().__init__()
        self.hopper = hopper
        self.intake = intake

        self.addRequirements(hopper)
        self.addRequirements(intake)

    def initialize(self):
        self.intake.set_state("jam_clear")
        self.hopper.set_state("jam_clear")

    def end(self, interrupted: bool):
        self.hopper.set_state("off")
        self.intake.set_state("stow")
