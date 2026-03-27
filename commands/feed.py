from commands2 import Command

from subsystems.hoppersubsystem import HopperSubsystem
from subsystems.launchersubsystem import LauncherSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain

from wpimath.geometry import Rotation2d

class Feed(Command):
    def __init__(self, launcher: LauncherSubsystem, hopper: HopperSubsystem,
                 intake: IntakeSubsystem, drive: CommandSwerveDrivetrain):
        super().__init__()
        self.launcher = launcher
        self.hopper = hopper
        self.intake = intake
        self.drive = drive

        self._launching_active = False

        self.addRequirements(launcher)
        self.addRequirements(hopper)
        self.addRequirements(intake)

    def initialize(self):
        self.launcher.set_state("standby")
        self.intake.set_state("deployed")
        self._launching_active = False
        self.drive.set_lookahead(True)
        self.drive.set_auto_slow(True)

    def execute(self):
        self.drive.set_clt_target_direction(Rotation2d.fromDegrees(self.drive.get_feed_alignment_heading(0.5)))
        self.launcher.set_target_by_range(max(self.drive.get_auto_lookahead_range_to_feed(0.5) - 0.75, 0))

        if self.launcher.get_at_target() and not self._launching_active:
            self.hopper.set_state("launching")
            self.intake.set_state("launching")

    def end(self, interrupted: bool):
        self.launcher.set_state("off")
        self.hopper.set_state("jam_clear")
        self.drive.set_lookahead(False)
        self.drive.set_auto_slow(False)
