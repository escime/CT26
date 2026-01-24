from commands2 import Command

from subsystems.intakesubsystem import IntakeSubsystem

class Intake(Command):

    def __init__(self, intake: IntakeSubsystem, hopper):  # TODO add hopper subsystem typecatch
        super().__init__()

        self._intake = intake
        self._hopper = hopper

    def initialize(self):
        self._intake.set_state("intake")
        # TODO add hopper set state here

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool):
        self._intake.set_state("deployed")
        # TODO add hopper set state here