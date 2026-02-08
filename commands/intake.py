from commands2 import Command

from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.hoppersubsystem import HopperSubsystem

class Intake(Command):

    def __init__(self, intake: IntakeSubsystem, hopper: HopperSubsystem):
        super().__init__()

        self._intake = intake
        self._hopper = hopper

        self._deploy_complete = False

        self.addRequirements(intake)
        self.addRequirements(hopper)

    def initialize(self):
        self._intake.set_state("intake")
        self._deploy_complete = False
        self._hopper.set_state("intaking")

    def execute(self):
        if self._intake.get_deployed() and not self._deploy_complete:
            self._intake.set_state("intaking")
            self._deploy_complete = True

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._intake.set_state("deployed")
        self._hopper.set_state("off")