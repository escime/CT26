from commands2 import Command

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.hoppersubsystem import HopperSubsystem
from subsystems.launchersubsystem import LauncherSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from phoenix6 import swerve
from wpimath.geometry import Rotation2d
from wpilib import DriverStation

class ManualLaunch(Command):
    """This is an advanced launch command. It implements shoot on the move using 3D pose tracking."""
    def __init__(self, drivetrain: CommandSwerveDrivetrain, launcher: LauncherSubsystem, hopper: HopperSubsystem,
                 intake: IntakeSubsystem, setpoint: str):
        super().__init__()
        self.drive = drivetrain
        self.launcher = launcher
        self.hopper = hopper
        self.intake = intake
        self.setpoint = setpoint

        self.brake = swerve.requests.SwerveDriveBrake()

        self._launching_active = False

        self.adder = 0

        self.addRequirements(launcher)
        self.addRequirements(hopper)
        self.addRequirements(intake)

    def initialize(self):
        self.launcher.set_state(self.setpoint)
        self.intake.set_state("deployed")
        self._launching_active = False

        self.adder = 0
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self.adder = 180

    def execute(self):
        if self.setpoint == "tower" and not self.get_clt_on_target():
            self.drive.set_clt_target_direction(Rotation2d.fromDegrees(self.drive.get_goal_alignment_heading(0.25)))
        else:
            self.drive.apply_request(lambda: self.brake).withTimeout(0.01).schedule()

        if self.launcher.get_at_target() and not self._launching_active:
            self.hopper.set_state("launching")
            self.intake.set_state("launching")
            self._launching_active = True

    def end(self, interrupted: bool):
        self.launcher.set_state("off")
        self.hopper.set_state("jam_clear")
        # self.intake.set_state("stow")

    def get_clt_on_target(self) -> bool:
        if self.drive.target_direction.degrees() - 1 < self.drive.get_pose().rotation().degrees() + self.adder < self.drive.target_direction.degrees() + 1:
            return True
        else:
            return False
