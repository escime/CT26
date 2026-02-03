from commands2.cmd import run, runOnce, runEnd
import wpilib.simulation
from commands2 import Command, button, SequentialCommandGroup, ParallelCommandGroup, ParallelRaceGroup, sysid, \
    InterruptionBehavior, ParallelDeadlineGroup, WaitCommand, ConditionalCommand

from constants import OIConstants
from subsystems.ledsubsystem3 import LEDSubsystem
from subsystems.utilsubsystem import UtilSubsystem

from wpilib import DriverStation, DataLogManager, Timer, Alert
from ntcore import NetworkTableInstance

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # Start master timer. ------------------------------------------------------------------------------------------
        self.timer = Timer()
        self.timer.start()

        # Configure network table access.
        self._inst = NetworkTableInstance.getDefault()
        self._auto_table = self._inst.getTable("Auto")
        self._debug_table = self._inst.getTable("Debug")

        # Configure button to enable robot logging.
        self.logging_button = self._debug_table.putBoolean("Logging Enabled?", False)

        # Startup subsystems. ------------------------------------------------------------------------------------------
        self.leds = LEDSubsystem()
        self.util = UtilSubsystem()

        # Setup driver & operator controllers. -------------------------------------------------------------------------
        self.driver_controller = button.CommandXboxController(OIConstants.kDriverControllerPort)
        self.operator_controller = button.CommandXboxController(OIConstants.kOperatorControllerPort)
        DriverStation.silenceJoystickConnectionWarning(True)
        self.test_bindings = False

        # Setup for all event-trigger commands. ------------------------------------------------------------------------
        self.configure_triggers()

    def configure_triggers(self) -> None:
        # Alliance win notifier light # TODO Test on the robot since it's not possible to simulate
        button.Trigger(lambda: self.util.get_game_data_received() and DriverStation.isTeleopEnabled()).onTrue(
            runOnce(lambda: self.leds.set_state("yellow_" + self.util.get_alliance_winner() + "_chaser"), self.leds)
        )
        self.driver_controller.a().onTrue(
            runOnce(lambda: self.leds.set_state("purple_flashing"), self.leds)
        ).onFalse(
            runOnce(lambda: self.leds.set_state("default"), self.leds)
        )
        self.driver_controller.b().onTrue(
            runOnce(lambda: self.leds.set_state("rainbow_chaser"), self.leds)
        ).onFalse(
            runOnce(lambda: self.leds.set_state("default"), self.leds)
        )
        self.driver_controller.x().onTrue(
            runOnce(lambda: self.leds.set_state("yellow_chaser"), self.leds)
        ).onFalse(
            runOnce(lambda: self.leds.set_state("default"), self.leds)
        )
        self.driver_controller.y().onTrue(
            runOnce(lambda: self.leds.set_state("white_flashing"), self.leds)
        ).onFalse(
            runOnce(lambda: self.leds.set_state("default"), self.leds)
        )

    def get_autonomous_command(self) -> Command:
        """Use this to pass the autonomous command to the main Robot class.
        Returns the command to run in autonomous
        """
        return None

    def enable_test_bindings(self, enabled: bool) -> None:
        self.test_bindings = enabled
