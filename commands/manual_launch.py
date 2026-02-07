from commands2 import Command

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.hoppersubsystem import HopperSubsystem
from subsystems.launchersubsystem import LauncherSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from phoenix6 import swerve

class ManualLaunch(Command):
    """This is an advanced launch command. It implements shoot on the move using 3D pose tracking."""
    def __init__(self, drivetrain: CommandSwerveDrivetrain, launcher: LauncherSubsystem, hopper: HopperSubsystem,
                 setpoint: str):
        super().__init__()
        self.drive = drivetrain
        self.launcher = launcher
        self.hopper = hopper
        self.setpoint = setpoint

        self.brake = swerve.requests.SwerveDriveBrake()

        self._launching_active = False

        self.addRequirements(launcher)
        self.addRequirements(hopper)

    def initialize(self):
        self.launcher.set_state(self.setpoint)
        self._launching_active = False

    def execute(self):
        # self.drive.apply_request(lambda: self.brake).withTimeout(0.01).schedule()

        if self.launcher.get_at_target() and not self._launching_active:
            self.hopper.set_state("launching")
            # print("LAUNCHING ENABLED")
        else:
            self._launching_active = False

    def end(self, interrupted: bool):
        self.launcher.set_state("off")
        self.hopper.set_state("off")
