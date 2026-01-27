from commands2 import Command

from subsystems.hoppersubsystem import HopperSubsystem
from subsystems.launchersubsystem import LauncherSubsystem
from subsystems.intakesubsystem import IntakeSubsystem

class Feed(Command):
    def __init__(self, launcher: LauncherSubsystem, hopper: HopperSubsystem,
                 intake: IntakeSubsystem):
        super().__init__()
        self.launcher = launcher
        self.hopper = hopper
        self.intake = intake

        self._launching_active = False

        self.addRequirements(launcher)
        self.addRequirements(hopper)
        self.addRequirements(intake)

    def initialize(self):
        self.launcher.set_state("feed")
        self.intake.set_state("deployed")
        self._launching_active = False

    def execute(self):
        if self.launcher.get_at_target() and not self._launching_active:
            self.hopper.set_state("launching")
            self.intake.set_state("launching")
        else:
            self._launching_active = False

    def end(self, interrupted: bool):
        self.launcher.set_state("off")
        self.hopper.set_state("off")
        self.intake.set_state("stow")
