from commands2 import Command

from subsystems.intakesubsystem import IntakeSubsystem

class EmergencyRetract(Command):

    def __init__(self, intake: IntakeSubsystem):
        super().__init__()

        self._intake = intake

        self.addRequirements(intake)

    def initialize(self):
        self._intake.set_state("retracting")

    def isFinished(self) -> bool:
        return self._intake.get_retracted()

    def end(self, interrupted: bool):
        self._intake.set_state("stow")
