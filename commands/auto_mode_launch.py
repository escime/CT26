from commands2 import Command
from wpimath.units import rotationsToDegrees

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.launchersubsystem import LauncherSubsystem
from subsystems.hoppersubsystem import HopperSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from phoenix6 import swerve
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Rotation2d

class AutoLaunch(Command):
    """This is a basic Launch command. Planned to be supplanted by a more advanced launch command that incorporates
    pose information."""
    def __init__(self, drivetrain: CommandSwerveDrivetrain, launcher: LauncherSubsystem, hopper: HopperSubsystem):
        super().__init__()
        self.drive = drivetrain
        self.launcher = launcher
        self.hopper = hopper

        self.brake = swerve.requests.SwerveDriveBrake()

        self.addRequirements(launcher)
        self.addRequirements(hopper)

    def initialize(self):
        self.drive.set_3d(True)
        self.drive.set_lookahead(True)
        self.launcher.set_state("standby")

    def execute(self):
        self.drive.set_clt_target_direction(Rotation2d.fromDegrees(self.drive.get_goal_alignment_heading(0.5)))
        self.drive.get_auto_lookahead_range_to_goal(0.1)
        self.drive.saved_request = (self.drive.clt_request.with_target_direction(self.drive.target_direction)
                                    .with_velocity_x(0)
                                    .with_velocity_y(0))
        if self.launcher.get_at_target():
            self.hopper.set_state("launching")

    def isFinished(self) -> bool:
        return True
