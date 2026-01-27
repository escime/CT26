from commands2 import Command

from subsystems.launchersubsystem import LauncherSubsystem
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.hoppersubsystem import HopperSubsystem

class AutoEndLaunch(Command):

    def __init__(self, launcher: LauncherSubsystem, drive: CommandSwerveDrivetrain, intake: IntakeSubsystem,
                 hopper: HopperSubsystem):
        super().__init__()
        self.launcher = launcher
        self.drive = drive
        self.hopper = hopper
        self.intake = intake

        self.addRequirements(launcher)
        self.addRequirements(hopper)
        self.addRequirements(intake)

    def initialize(self):
        self.launcher.set_state("off")
        self.drive.set_3d(True)
        self.drive.set_lookahead(False)
        self.intake.set_state("stow")
        self.hopper.set_state("off")

    def isFinished(self) -> bool:
        return True
