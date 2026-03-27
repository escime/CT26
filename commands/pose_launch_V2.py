from commands2 import Command

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.hoppersubsystem import HopperSubsystem
from subsystems.launchersubsystem import LauncherSubsystem
from subsystems.intakesubsystem import IntakeSubsystem
from subsystems.utilsubsystem import UtilSubsystem
from phoenix6 import swerve
from wpimath.geometry import Rotation2d
from wpilib import DriverStation, SmartDashboard
from wpimath.controller import ProfiledPIDController, PIDController
from wpimath.trajectory import TrapezoidProfile, ExponentialProfileMeterVolts
from wpimath.units import rotationsToDegrees
from commands2.button import CommandXboxController
from generated.tuner_constants import TunerConstants
from math import sqrt, pow


class PoseLaunchV2(Command):
    """This is an advanced launch command. It implements shoot on the move using 3D pose tracking."""
    def __init__(self, drivetrain: CommandSwerveDrivetrain, launcher: LauncherSubsystem, hopper: HopperSubsystem,
                 intake: IntakeSubsystem, util: UtilSubsystem, joystick: CommandXboxController):
        super().__init__()
        self.drive = drivetrain
        self.launcher = launcher
        self.hopper = hopper
        self.intake = intake
        self.util = util
        self._joystick = joystick

        self._launching_active = False

        self.rotation_request = (swerve.requests.FieldCentric()
                                 # .with_velocity_y(0)
                                 # .with_velocity_x(0)
                                 .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY))

        kp = 0.029 # This is what needs tuned
        ki = 0
        kd = 0
        vel = 540
        accl = 300

        self.rotation_controller = ProfiledPIDController(kp, ki, kd,
                                                         TrapezoidProfile.Constraints(vel, accl),
                                                         0.04)
        self.rotation_controller.enableContinuousInput(-180, 180)
        self.rotation_controller.setTolerance(1)


        # self.rotation_controller = PIDController(kp, 0, 0, 0.04)
        # self.profile = ExponentialProfileMeterVolts(ExponentialProfileMeterVolts.Constraints.fromCharacteristics(1, 0.1, 0))
        # self.setpoint = ExponentialProfileMeterVolts.State(0, 0)

        SmartDashboard.putNumber("PLV2 Kp", kp)
        SmartDashboard.putNumber("PLV2 Ki", ki)
        SmartDashboard.putNumber("PLV2 Kd", kd)
        SmartDashboard.putNumber("PLV2 vel", vel)
        SmartDashboard.putNumber("PLV2 accl", accl)

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

        self.rotation_controller.setPID(SmartDashboard.getNumber("PLV2 Kp", 0.01),
                                        SmartDashboard.getNumber("PLV2 Ki", 0),
                                        SmartDashboard.getNumber("PLV2 Kd", 0))
        self.rotation_controller.setConstraints(TrapezoidProfile.Constraints(SmartDashboard.getNumber("PLV2 vel", 1),
                                                                             SmartDashboard.getNumber("PLV2 accl", 1)))

        self._launching_active = False

        self.rotation_controller.reset(self.drive.get_pose().rotation().degrees())
        # self.rotation_controller.reset()

        self.drive.set_slow_mode(0.375, 0.375)

        if sqrt(pow(self.drive.get_chassis_speeds().vx, 2) + pow(self.drive.get_chassis_speeds().vy, 2)) >= 0.1 * 5.12:
            self.rotation_controller.setTolerance(10)
        else:
            self.rotation_controller.setTolerance(3)

    def execute(self):
        # self.drive.set_clt_target_direction(Rotation2d.fromDegrees(self.drive.get_goal_alignment_heading_with_tof(0.25)))
        self.launcher.set_target_by_range(self.drive.get_auto_lookahead_range_with_tof(0.25))
        rotation_target = self.drive.get_goal_alignment_heading_with_tof(0.25) + 180

        rotate_output = self.rotation_controller.calculate(self.drive.get_pose().rotation().degrees(), rotation_target)
        # goal_state = ExponentialProfileMeterVolts.State(rotation_target, 0)
        # next_goal = self.profile.calculate(0.04, self.setpoint, goal_state)

        SmartDashboard.putNumber("PLV2 Kp", self.rotation_controller.getP())
        SmartDashboard.putNumber("PLV2 vel", self.rotation_controller.getConstraints().maxVelocity)
        SmartDashboard.putNumber("PLV2 accl", self.rotation_controller.getConstraints().maxAcceleration)

        self.drive.apply_request(lambda: (self.rotation_request
                                          .with_rotational_rate(rotate_output)
                                          .with_velocity_x(self._joystick.getLeftY() * TunerConstants.speed_at_12_volts * -1 * self.drive.slow_mode_application)
                                          .with_velocity_y(self._joystick.getLeftX() * TunerConstants.speed_at_12_volts * -1 * self.drive.slow_mode_application))).withTimeout(0.02).schedule()

        if self.launcher.get_at_target() and not self._launching_active and self.util.get_hub_active() and self.rotation_controller.atSetpoint():
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
        self.drive.set_slow_mode(1, 1)
        self.launcher.set_state("off")
        self.hopper.set_state("jam_clear")
        # self.intake.set_state("stow")
        # self.drive.set_3d(False)
        self.drive.set_lookahead(False)
        self.drive.set_auto_slow(False)