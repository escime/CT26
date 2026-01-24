from commands2 import Command

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.launchersubsystem import LauncherSubsystem
from phoenix6 import swerve
from wpimath.geometry import Rotation2d
from math import pi

class PoseLaunch(Command):
    """This is an advanced launch command. It implements shoot on the move using 3D pose tracking."""
    def __init__(self, drivetrain: CommandSwerveDrivetrain, launcher: LauncherSubsystem):
        super().__init__()
        self.drive = drivetrain
        self.launcher = launcher
        self.brake = swerve.requests.SwerveDriveBrake()

        self.addRequirements(launcher)

    def initialize(self):
        self.launcher.set_state("standby")
        self.drive.set_3d(True)
        self.drive.set_lookahead(True)
        self.drive.set_auto_slow(True)

    def execute(self):
        self.drive.set_clt_target_direction(Rotation2d.fromDegrees(self.drive.get_goal_alignment_heading(0.5)))
        self.launcher.set_target_by_range(self.drive.get_auto_lookahead_range_to_goal(0.5))

    def end(self, interrupted: bool):
        self.launcher.set_state("off")
        self.drive.set_3d(False)
        self.drive.set_lookahead(False)
        self.drive.set_auto_slow(False)

    # def convert_hood_angle_to_degrees(self, hood_angle: float) -> float:
    #     return 170 - ( 35 * hood_angle )
    #
    # def convert_RPM_to_v0(self, rpm: float) -> float:
    #     return rpm * pi * 4 * 0.0254 / 60
    #
    # def calculate_time_to_goal(self, distance: float) -> float:
    #     return (distance / () ) + 0.1