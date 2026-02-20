from commands2 import Command

from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.launchersubsystem import LauncherSubsystem
from subsystems.hoppersubsystem import HopperSubsystem
from subsystems.climbersubsystem import ClimberSubsystem
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from telemetry import Telemetry

class DebugMode(Command):

    def __init__(self,
                 telemetry: Telemetry,
                 intake: IntakeSubsystem,
                 launcher: LauncherSubsystem,
                 hopper: HopperSubsystem,
                 # climber: ClimberSubsystem,
                 drive: CommandSwerveDrivetrain,
                 on: bool):
        super().__init__()
        self._intake = intake
        self._launcher = launcher
        self._hopper = hopper
        self._on = on
        self._telemetry = telemetry
        # self._climber = climber
        self._drive = drive

    def initialize(self):
        self._intake.set_debug_mode(self._on)
        self._launcher.set_debug_mode(self._on)
        self._hopper.set_debug_mode(self._on)
        self._telemetry.set_debug_mode(self._on)
        # self._climber.set_debug_mode(self._on)
        self._drive.set_debug_mode(self._on)

    def isFinished(self) -> bool:
        return True
