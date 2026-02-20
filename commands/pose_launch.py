from commands2 import Command

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.hoppersubsystem import HopperSubsystem
from subsystems.launchersubsystem import LauncherSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.utilsubsystem import UtilSubsystem
from phoenix6 import swerve
from wpimath.geometry import Rotation2d
from wpilib import DriverStation, SmartDashboard


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

        self._launching_active = False
        self.adder = 0

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

        self.adder = 0
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.adder = 180

    def execute(self):
        self.drive.set_clt_target_direction(Rotation2d.fromDegrees(self.drive.get_goal_alignment_heading_with_tof(0.25)))
        self.launcher.set_target_by_range(self.drive.get_auto_lookahead_range_with_tof(0.25))
        SmartDashboard.putNumber("Range to Goal", self.drive.get_auto_lookahead_range_with_tof(0.25))

        if self.launcher.get_at_target() and not self._launching_active and self.util.get_hub_active() and self.get_clt_on_target():
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
            pass

    def end(self, interrupted: bool):
        self.launcher.set_state("off")
        self.hopper.set_state("off")
        # self.intake.set_state("stow")
        self.drive.set_3d(False)
        self.drive.set_lookahead(False)
        self.drive.set_auto_slow(False)

    def get_clt_on_target(self) -> bool:
        if self.drive.target_direction.degrees() - 2 < self.drive.get_pose().rotation().degrees() + self.adder < self.drive.target_direction.degrees() + 2:
            return True
        else:
            return False
