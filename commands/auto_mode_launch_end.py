from commands2 import Command

from subsystems.launchersubsystem import LauncherSubsystem
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain

class AutoEndLaunch(Command):

    def __init__(self, launcher: LauncherSubsystem, drive: CommandSwerveDrivetrain):
        super().__init__()
        self.launcher = launcher
        self.drive = drive

        self.addRequirements(launcher)

    def initialize(self):
        self.launcher.set_state("off")
        self.drive.set_3d(True)
        self.drive.set_lookahead(False)

    def isFinished(self) -> bool:
        return True
