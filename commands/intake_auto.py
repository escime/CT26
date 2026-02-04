from commands2 import Command

from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.hoppersubsystem import HopperSubsystem

class IntakeAuto(Command):

    def __init__(self, intake: IntakeSubsystem, hopper: HopperSubsystem):
        super().__init__()

        self._intake = intake
        self._hopper = hopper

    def initialize(self):
        self._intake.set_state("intake")
        self._hopper.set_state("intaking")

    def isFinished(self) -> bool:
        return True
