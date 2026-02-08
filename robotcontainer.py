from commands2.cmd import run, runOnce, runEnd
import wpilib.simulation
from commands2 import Command, button, SequentialCommandGroup, ParallelCommandGroup, ParallelRaceGroup, sysid, \
    InterruptionBehavior, ParallelDeadlineGroup, WaitCommand, ConditionalCommand

from commands.manual_launch import ManualLaunch
from constants import OIConstants
from subsystems.ledsubsystem3 import LEDSubsystem
from subsystems.utilsubsystem import UtilSubsystem
from subsystems.command_swerve_drivetrain import ResetCLT, SetRotation, SetCLTTarget
from subsystems.launchersubsystem import LauncherSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.hoppersubsystem import HopperSubsystem
from subsystems.climbersubsystem import ClimberSubsystem

from wpilib import SmartDashboard, SendableChooser, DriverStation, DataLogManager, Timer, Alert, Joystick, \
    XboxController
from wpimath.filter import SlewRateLimiter
from pathplannerlib.auto import NamedCommands, AutoBuilder

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve, SignalLogger
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians
from ntcore import NetworkTableInstance

from math import pi, pow, copysign, atan2

from commands.baseline import Baseline
from commands.check_drivetrain import CheckDrivetrain
from commands.start_auto_timer import StartAutoTimer
from commands.stop_auto_timer import StopAutoTimer
from commands.pathfollowing_endpoint import PathfollowingEndpointClose
from commands.wheel_radius_calculator import WheelRadiusCalculator
from commands.launch import Launch
from commands.pose_launch import PoseLaunch
from commands.intake import Intake
from commands.auto_mode_launch import AutoLaunch
from commands.auto_mode_launch_end import AutoEndLaunch
from commands.auto_climb import AutoClimb
from commands.feed import Feed
from commands.outpost_feed import OutpostFeed
from commands.jam_clear import JamClear
from commands.intake_auto import IntakeAuto
from commands.emergency_retract import EmergencyRetract
from commands.test_launch import TestLaunch

# Controller layout: https://padcrafter.com/?templates=CT26+Driver+Controller%2C+TELEOP%7CCT26+Driver+Controller%2C+TEST&col=%23D3D3D3%2C%233E4B50%2C%23FFFFFF&leftStick=Translate+%28CLT%29%7CTranslate&rightStick=Rotate+%28CLT%29&rightTrigger=%28HOLD%29+Slow+Mode%7C%28HOLD%29+Set+SysID+to+Translation&dpadUp=POV+Snap+North&dpadRight=POV+Snap+East&dpadLeft=POV+Snap+West&dpadDown=POV+Snap+South&yButton=Reset+Pose%7C%28HOLD%29+Run+Quasistatic+Forward&leftTrigger=%28HOLD%29+Brake+Mode&plat=%7C%7C0&startButton=%7C%28HOLD%29+Point+Modules&backButton=%7CCalculate+Wheel+Radius&rightBumper=%7C%28HOLD%29+Set+SysID+to+Rotation&leftBumper=%7C%28HOLD%29+Set+SysID+to+Steer&xButton=%7C%28HOLD%29+Run+Dynamic+Reverse&bButton=%7C%28HOLD%29+Run+Quasistatic+Reverse&aButton=%7C%28HOLD%29+Run+Dynamic+Forward&rightStickClick=%7CRotate

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

        # Disable automatic CTR logging
        SignalLogger.enable_auto_logging(False)

        # Configure system logging. ------------------------------------------------------------------------------------
        self.alert_logging_enabled = Alert("Robot Logging is Enabled", Alert.AlertType.kWarning)
        if wpilib.RobotBase.isReal():
            if self._debug_table.getBoolean("Logging Enabled?", False) is True:
                DataLogManager.start()
                DriverStation.startDataLog(DataLogManager.getLog(), True)
                SignalLogger.start()
                self.alert_logging_enabled.set(True)
            else:
                SignalLogger.stop()
        else:
            SignalLogger.stop()

        # Startup subsystems. ------------------------------------------------------------------------------------------
        self.leds = LEDSubsystem()
        # self.leds = LEDs(self.timer)
        self.util = UtilSubsystem()
        self.launcher = LauncherSubsystem()
        self.intake = IntakeSubsystem()
        self.hopper = HopperSubsystem()
        self.climber = ClimberSubsystem()

        # Setup driver & operator controllers. -------------------------------------------------------------------------
        self.driver_controller = button.CommandXboxController(OIConstants.kDriverControllerPort)
        self.operator_controller = button.CommandXboxController(OIConstants.kOperatorControllerPort)
        DriverStation.silenceJoystickConnectionWarning(True)
        self.test_bindings = False

        # Configure drivetrain settings. -------------------------------------------------------------------------------
        self._max_speed = TunerConstants.speed_at_12_volts  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(0.75)  # 3/4 of a rotation per second max angular velocity

        self._logger = Telemetry(self._max_speed)

        self.drivetrain = TunerConstants.create_drivetrain()

        self._drive = (
            swerve.requests.FieldCentric()  # I want field-centric
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.1)  # Add a 10% deadband
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_desaturate_wheel_speeds(True)
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._hold_heading = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._max_speed * 0.1)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY)
            .with_desaturate_wheel_speeds(True)
        )
        self._hold_heading.heading_controller.setPID(5, 0, 0)
        self._hold_heading.heading_controller.enableContinuousInput(0, -2 * pi)
        self._hold_heading.heading_controller.setTolerance(0.1)  # 0.1

        # Register commands for PathPlanner. ---------------------------------------------------------------------------
        self.registerCommands()

        self._auto_table.putBoolean("Misalignment Indicator Active?", False)
        self._auto_table.putNumber("Misalignment Angle", 0)

        # Setup for all event-trigger commands. ------------------------------------------------------------------------
        # self.configureTriggersSmartDash()
        self.configure_test_bindings()
        self.configure_triggers()

        # Setup autonomous selector on the dashboard. ------------------------------------------------------------------
        self.m_chooser = AutoBuilder.buildAutoChooser("DoNothing")
        SmartDashboard.putData("Auto Select", self.m_chooser)

        self.drive_filter_x = SlewRateLimiter(3, -3, 0)
        self.drive_filter_y = SlewRateLimiter(3, -3, 0)

    def configure_triggers(self) -> None:
        # DRIVER COMMANDS # ############################################################################################
        # Drive in Closed-Loop Turning Mode.
        self.drivetrain.setDefaultCommand(
                self.drivetrain.apply_request(
                    lambda: (
                        self.drivetrain.drive_clt(
                            self.drive_filter_y.calculate(
                                self.deadband_controller(self.driver_controller.getLeftY())) * self._max_speed * -1,
                            self.drive_filter_x.calculate(
                                self.deadband_controller(self.driver_controller.getLeftX())) * self._max_speed * -1,
                            self.deadband_controller(self.driver_controller.getRightX()) * -1
                        )
                    )
                )
            )

        self.driver_controller.axisMagnitudeGreaterThan(4, 0.04).and_(lambda: not self.test_bindings).and_(self.driver_controller.rightTrigger().negate()).whileTrue(
            self.drivetrain.apply_request(
                lambda: (
                    self._drive
                    .with_velocity_x(self.drive_filter_y.calculate(self.deadband_controller(self.driver_controller.getLeftY())) * -1 * self._max_speed)
                    .with_velocity_y(self.drive_filter_x.calculate(self.deadband_controller(self.driver_controller.getLeftX())) * -1 * self._max_speed)
                    .with_rotational_rate(self.deadband_controller(self.driver_controller.getRightX()) * -1 * self._max_angular_rate)
                )
            )
        ).onFalse(
            ResetCLT(self.drivetrain)
        )

        # Disable CLT, activate open-loop driving.
        self.driver_controller.leftStick().toggleOnTrue(
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -copysign(pow(self.drive_filter_x.calculate(self.driver_controller.getLeftY()), 1),
                                  self.drive_filter_x.calculate(self.driver_controller.getLeftY()))
                        * self._max_speed)
                    .with_velocity_y(-copysign(pow(self.drive_filter_y.calculate(self.driver_controller.getLeftX()), 1),
                                               self.drive_filter_y.calculate(self.driver_controller.getLeftX()))
                                     * self._max_speed)
                    .with_rotational_rate(-copysign(pow(self.driver_controller.getRightX(), 1),
                                                    self.driver_controller.getRightX())
                                          * self._max_angular_rate)
                )
            )
        ).onFalse(
            ResetCLT(self.drivetrain)
        )

        # Automatic Launch command.
        self.driver_controller.rightTrigger(0.25).and_(lambda: not self.test_bindings).whileTrue(
            ParallelCommandGroup(
                PoseLaunch(self.drivetrain, self.launcher, self.hopper, self.intake, self.util),
                runOnce(lambda: self.leds.set_state("yellow_chaser"), self.leds)
            )
        ).onFalse(
            SequentialCommandGroup(
                ResetCLT(self.drivetrain),
                runOnce(lambda: self.leds.set_state("default"), self.leds)
            )
        )

        # Manual launch commands.
        self.driver_controller.leftBumper().and_(lambda: not self.test_bindings).whileTrue(
            ManualLaunch(self.drivetrain, self.launcher, self.hopper, self.intake, "hub")
        ).onFalse(
            SequentialCommandGroup(
                EmergencyRetract(self.intake),
                WaitCommand(0.5),
                runOnce(lambda: self.hopper.set_state("off"), self.hopper),
            )
        )
        self.driver_controller.rightBumper().and_(lambda: not self.test_bindings).whileTrue(
            ManualLaunch(self.drivetrain, self.launcher, self.hopper, self.intake, "tower")
        ).onFalse(
            SequentialCommandGroup(
                EmergencyRetract(self.intake),
                WaitCommand(0.5),
                runOnce(lambda: self.hopper.set_state("off"), self.hopper)
            )
        )
        self.driver_controller.povLeft().and_(lambda: not self.test_bindings).onTrue(
            runOnce(lambda: self.launcher.live_reconfigure(), self.launcher).ignoringDisable(True)
        )
        self.driver_controller.povRight().and_(lambda: not self.test_bindings).whileTrue(
            TestLaunch(self.drivetrain, self.launcher, self.hopper, self.intake)
        ).onFalse(
            SequentialCommandGroup(
                EmergencyRetract(self.intake),
                WaitCommand(0.5),
                runOnce(lambda: self.hopper.set_state("off"), self.hopper)
            )
        )

        # Intake command.
        self.driver_controller.leftTrigger(0.25).and_(lambda: not self.test_bindings).whileTrue(
            ParallelCommandGroup(
                Intake(self.intake, self.hopper),
                runOnce(lambda: self.leds.set_state("purple_flashing"), self.leds)
            )
        ).onFalse(
            runOnce(lambda: self.leds.set_state("default"), self.leds)
        )

        # Climber deploy.
        self.driver_controller.povUp().and_(lambda: not self.test_bindings).onTrue(
            SequentialCommandGroup(
                SetCLTTarget(self.drivetrain, Rotation2d.fromDegrees(180)),
                runOnce(lambda: self.climber.set_state("deployed"), self.climber),
                runOnce(lambda: self.leds.set_state("rainbow_chaser"), self.leds)
            )
        )

        # Climber climb.
        self.driver_controller.povDown().and_(lambda: not self.test_bindings).onTrue(
            runOnce(lambda: self.climber.set_state("climb"), self.climber)
        )

        # Stow robot for crossing the BUMP or TRENCH.
        self.driver_controller.a().and_(lambda: not self.test_bindings).onTrue(
            SequentialCommandGroup(
                SetCLTTarget(self.drivetrain, Rotation2d.fromDegrees(135)),
                runOnce(lambda: self.launcher.set_state("safety"), self.launcher),
                runOnce(lambda: self.climber.set_state("stow"), self.climber),
                runOnce(lambda: self.leds.set_state("default"), self.leds)
            )
        )

        # Emergency intake retraction.
        self.driver_controller.b().and_(lambda: not self.test_bindings).onTrue(
            EmergencyRetract(self.intake)
        )

        # Feed button (does not rotate drivetrain at all).
        self.driver_controller.x().and_(lambda: not self.test_bindings).whileTrue(
            Feed(self.launcher, self.hopper, self.intake)
        )

        # Outpost feed button.
        self.driver_controller.rightStick().and_(lambda: not self.test_bindings).whileTrue(
            ParallelCommandGroup(
                OutpostFeed(self.launcher, self.hopper, self.intake),
                runOnce(lambda: self.leds.set_state("white_flashing"), self.leds)
            )
        ).onFalse(
            SequentialCommandGroup(
                WaitCommand(0.5),
                runOnce(lambda: self.hopper.set_state("off"), self.hopper),
                runOnce(lambda: self.leds.set_state("default"), self.leds)
            )
        )

        # Auto jam clear.
        self.driver_controller.back().and_(lambda: not self.test_bindings).whileTrue(
            JamClear(self.hopper, self.intake)
        )

        # Alliance win notifier light
        button.Trigger(lambda: self.util.get_game_data_received() and DriverStation.isTeleopEnabled()).toggleOnTrue(
            runOnce(lambda: self.leds.set_state("yellow_" + self.util.get_alliance_winner() + "_chaser"),
                    self.leds)
        )

        # Auto climbing.
        self.driver_controller.start().and_(lambda: not self.test_bindings).whileTrue(
            SequentialCommandGroup(
                AutoClimb(self.drivetrain, self.climber),
                ResetCLT(self.drivetrain)
            )
        )

        # Reset pose.
        self.driver_controller.y().and_(lambda: not self.test_bindings).onTrue(
            SequentialCommandGroup(
                runOnce(lambda: self.drivetrain.reset_odometry(), self.drivetrain).ignoringDisable(True),
                ResetCLT(self.drivetrain).ignoringDisable(True)
            )
        )

        # OPERATOR COMMANDS # ##########################################################################################
        self.operator_controller.a().onTrue(
            runOnce(lambda: self.launcher.set_state("standby"), self.launcher)
        ).onFalse(
            runOnce(lambda: self.launcher.set_state("off"), self.launcher)
        )
        self.operator_controller.b().onTrue(
            runOnce(lambda: self.hopper.set_state("launching"), self.hopper)
        ).onFalse(
            runOnce(lambda: self.hopper.set_state("off"), self.hopper)
        )

        # Configuration for telemetry.
        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )


    def get_autonomous_command(self) -> Command:
        """Use this to pass the autonomous command to the main Robot class.
        Returns the command to run in autonomous
        """
        return self.m_chooser.getSelected()

    def configure_test_bindings(self) -> None:
        self.configure_sys_id()

        # Point all modules in a direction
        self.driver_controller.start().and_(lambda: self.test_bindings).whileTrue(self.drivetrain.apply_request(
            lambda: self._point.with_module_direction(
                Rotation2d(-1 * self.driver_controller.getLeftY()
                           - 1 * self.driver_controller.getLeftX()))))

        self.driver_controller.back().and_(lambda: self.test_bindings).onTrue(
            WheelRadiusCalculator(self.drivetrain, self.timer)
        )

    def configure_sys_id(self) -> None:
        (self.driver_controller.y().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_quasistatic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.b().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_quasistatic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.a().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_dynamic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.x().and_(lambda: self.test_bindings).and_(self.driver_controller.rightTrigger())
         .whileTrue(self.drivetrain.sys_id_translation_dynamic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.y().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_quasistatic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.b().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_quasistatic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.a().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_dynamic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.x().and_(lambda: self.test_bindings).and_(self.driver_controller.rightBumper())
         .whileTrue(self.drivetrain.sys_id_rotation_dynamic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.y().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_quasistatic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.b().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_quasistatic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.a().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_dynamic(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.x().and_(lambda: self.test_bindings).and_(self.driver_controller.leftBumper())
         .whileTrue(self.drivetrain.sys_id_steer_dynamic(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.y().and_(lambda: self.test_bindings).and_(self.driver_controller.leftTrigger())
         .whileTrue(self.launcher.sys_id_quasistatic_leader(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.b().and_(lambda: self.test_bindings).and_(self.driver_controller.leftTrigger())
         .whileTrue(self.launcher.sys_id_quasistatic_leader(sysid.SysIdRoutine.Direction.kReverse)))
        (self.driver_controller.a().and_(lambda: self.test_bindings).and_(self.driver_controller.leftTrigger())
         .whileTrue(self.launcher.sys_id_dynamic_leader(sysid.SysIdRoutine.Direction.kForward)))
        (self.driver_controller.x().and_(lambda: self.test_bindings).and_(self.driver_controller.leftTrigger())
         .whileTrue(self.launcher.sys_id_dynamic_leader(sysid.SysIdRoutine.Direction.kReverse)))

    def enable_test_bindings(self, enabled: bool) -> None:
        self.test_bindings = enabled

    def check_endpoint_closed(self) -> bool:
        return self.drivetrain.endpoint[0] - 0.02 < self.drivetrain.get_pose().x < self.drivetrain.endpoint[
            0] + 0.02 and self.drivetrain.endpoint[
            1] - 0.02 < self.drivetrain.get_pose().y < self.drivetrain.endpoint[1] + 0.02

    def deadband_controller(self, joystick_input) -> float:
        if -0.05 < joystick_input < 0.05:
            return 0
        else:
            return joystick_input

    def registerCommands(self):
        # NamedCommands.registerCommand("rainbow_leds", runOnce(lambda: self.leds.set_state("rainbow"),
        #                                                       self.leds))
        # NamedCommands.registerCommand("flash_green",
        #                               SequentialCommandGroup(
        #                                   runOnce(lambda: self.leds.set_flash_color_color([255, 0, 0]),
        #                                           self.leds),
        #                                   runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
        #                                   runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        # NamedCommands.registerCommand("flash_red",
        #                               SequentialCommandGroup(
        #                                   runOnce(lambda: self.leds.set_flash_color_color([0, 255, 0]),
        #                                           self.leds),
        #                                   runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
        #                                   runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        # NamedCommands.registerCommand("flash_blue",
        #                               SequentialCommandGroup(
        #                                   runOnce(lambda: self.leds.set_flash_color_color([0, 0, 255]),
        #                                           self.leds),
        #                                   runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
        #                                   runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        # NamedCommands.registerCommand("flash_purple",
        #                               SequentialCommandGroup(
        #                                   runOnce(lambda: self.leds.set_flash_color_color([50, 149, 168]),
        #                                           self.leds),
        #                                   runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
        #                                   runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        # NamedCommands.registerCommand("flash_yellow",
        #                               SequentialCommandGroup(
        #                                   runOnce(lambda: self.leds.set_flash_color_color([255, 255, 0]),
        #                                           self.leds),
        #                                   runOnce(lambda: self.leds.set_flash_color_rate(2), self.leds),
        #                                   runOnce(lambda: self.leds.set_state("flash_color"), self.leds)))
        # NamedCommands.registerCommand("default_leds", runOnce(lambda: self.leds.set_state("default"),
        #                                                       self.leds))
        NamedCommands.registerCommand("baseline", Baseline(self.drivetrain, self.timer))
        NamedCommands.registerCommand("check_drivetrain", CheckDrivetrain(self.drivetrain, self.timer))
        NamedCommands.registerCommand("override_heading_goal",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.drivetrain.set_lookahead(True)),
                                          runOnce(lambda: self.drivetrain.set_pathplanner_rotation_override("goal"))
                                        )
                                      )
        NamedCommands.registerCommand("override_heading_gp",
                                      runOnce(lambda: self.drivetrain.set_pathplanner_rotation_override("gp")))
        NamedCommands.registerCommand("disable_override_heading",
                                      SequentialCommandGroup(
                                          runOnce(lambda: self.drivetrain.set_lookahead(False)),
                                          runOnce(lambda: self.drivetrain.set_pathplanner_rotation_override("none"))
                                      ))
        NamedCommands.registerCommand("start_timer", StartAutoTimer(self.util, self.timer))
        NamedCommands.registerCommand("stop_timer", StopAutoTimer(self.util, self.timer))
        NamedCommands.registerCommand("reset_CLT", ResetCLT(self.drivetrain))
        NamedCommands.registerCommand("intake_on", IntakeAuto(self.intake, self.hopper))
        NamedCommands.registerCommand("3d_mode_on", runOnce(lambda: self.drivetrain.set_3d(True)))
        NamedCommands.registerCommand("3d_mode_off", runOnce(lambda: self.drivetrain.set_3d(True)))
        NamedCommands.registerCommand("climb", runOnce(lambda: self.climber.set_state("climb"), self.climber))
        NamedCommands.registerCommand("climb_deploy", runOnce(lambda: self.climber.set_state("deployed"), self.climber))
        NamedCommands.registerCommand("standby_flywheel", runOnce(lambda: self.launcher.set_state("standby"), self.launcher))
        NamedCommands.registerCommand(
            "launch",
            SequentialCommandGroup(
                SequentialCommandGroup(
                    AutoLaunch(self.drivetrain, self.launcher, self.hopper, self.intake),
                    self.drivetrain.apply_request(lambda: self.drivetrain.saved_request).withTimeout(0.02)
                ).repeatedly().withTimeout(4), # TODO change to last shot condition
            AutoEndLaunch(self.launcher, self.drivetrain, self.intake, self.hopper),
            ResetCLT(self.drivetrain)
            )
        )
