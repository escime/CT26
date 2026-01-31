from commands2 import Command

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.hoppersubsystem import HopperSubsystem
from subsystems.launchersubsystem import LauncherSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.utilsubsystem import UtilSubsystem
from phoenix6 import swerve
from wpimath.geometry import Rotation2d

class PoseLaunch(Command):
    """This is an advanced launch command. It implements shoot on the move using 3D pose tracking."""
    def __init__(self, drivetrain: CommandSwerveDrivetrain, launcher: LauncherSubsystem, hopper: HopperSubsystem,
                 intake: IntakeSubsystem, util: UtilSubsystem):
        super().__init__()
        self.drive = drivetrain
        self.launcher = launcher
        self.hopper = hopper
        self.intake = intake
        self.util = util

        self.brake = swerve.requests.SwerveDriveBrake()

        self._launching_active = False

        self.addRequirements(launcher)
        self.addRequirements(hopper)
        self.addRequirements(intake)
        self.addRequirements(util)

    def initialize(self):
        self.launcher.set_state("standby")
        self.intake.set_state("deployed")
        self.drive.set_3d(True)
        self.drive.set_lookahead(True)
        self.drive.set_auto_slow(True)

        self._launching_active = False

    def execute(self):
        self.drive.set_clt_target_direction(Rotation2d.fromDegrees(self.drive.get_goal_alignment_heading(0.5)))
        self.launcher.set_target_by_range(self.drive.get_auto_lookahead_range_to_goal(0.5))

        if self.launcher.get_at_target() and not self._launching_active and self.util.get_hub_active():
            self.hopper.set_state("launching")
            self.intake.set_state("launching")
            self._launching_active = True
        elif self._launching_active and not self.util.get_hub_active():
            self.hopper.set_state("off")
            self.intake.set_state("deployed")
            self._launching_active = False
        elif self._launching_active and self.util.get_hub_active():
            pass
        else:
            self._launching_active = False

    def end(self, interrupted: bool):
        self.launcher.set_state("off")
        self.hopper.set_state("off")
        self.intake.set_state("stow")
        self.drive.set_3d(False)
        self.drive.set_lookahead(False)
        self.drive.set_auto_slow(False)
