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
from wpilib import DriverStation

class AutoLaunch(Command):
    """This is a basic Launch command. Planned to be supplanted by a more advanced launch command that incorporates
    pose information."""
    def __init__(self, drivetrain: CommandSwerveDrivetrain, launcher: LauncherSubsystem, hopper: HopperSubsystem,
                 intake: IntakeSubsystem):
        super().__init__()
        self.drive = drivetrain
        self.launcher = launcher
        self.hopper = hopper
        self.intake = intake

        self.adder = 0

        self.addRequirements(launcher)
        self.addRequirements(hopper)
        self.addRequirements(intake)

    def initialize(self):
        self.drive.set_3d(True)

        self.adder = 0
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.adder = 180

    def execute(self):
        self.drive.set_clt_target_direction(Rotation2d.fromDegrees(self.drive.get_goal_alignment_heading_with_tof(0.25)))
        self.launcher.set_target_by_range(self.drive.get_auto_lookahead_range_with_tof(0.25))
        self.drive.saved_request = (self.drive.clt_request.with_target_direction(self.drive.target_direction)
                                    .with_velocity_x(0)
                                    .with_velocity_y(0))
        if self.launcher.get_at_target() and self.get_clt_on_target():
            self.hopper.set_state("launching")
            self.intake.set_state("launching")


    def isFinished(self) -> bool:
        return True

    def get_clt_on_target(self) -> bool:
        if self.drive.target_direction.degrees() - 2 < self.drive.get_pose().rotation().degrees() + self.adder < self.drive.target_direction.degrees() + 2:
            return True
        else:
            return False
