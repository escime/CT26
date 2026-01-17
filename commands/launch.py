from commands2 import Command
from wpimath.units import rotationsToDegrees

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.launchersubsystem import LauncherSubsystem
from phoenix6 import swerve
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Rotation2d
from wpilib import DriverStation

class Launch(Command):
    """This is a basic Launch command. Planned to be supplanted by a more advanced launch command that incorporates
    pose information."""
    def __init__(self, drivetrain: CommandSwerveDrivetrain, launcher: LauncherSubsystem):
        super().__init__()
        self.drive = drivetrain
        self.launcher = launcher

        self.rotation_request = (swerve.requests.RobotCentric()
                                 .with_velocity_x(0)
                                 .with_velocity_y(0)
                                 .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY))

        self.rotation_controller = ProfiledPIDController(0.13, 0, 0,
                                                         TrapezoidProfile.Constraints(rotationsToDegrees(0.75),
                                                                                      rotationsToDegrees(0.1)))

        self.rotation_controller.enableContinuousInput(-180, 180)
        self.rotation_controller.setTolerance(1)

        self.brake = swerve.requests.SwerveDriveBrake()

    def initialize(self):
        self.launcher.set_state("standby")

    def execute(self):
        if self.drive.target_in_view:
            rotation_output = self.rotation_controller.calculate(self.drive.target_yaw, 0)
            self.launcher.set_target_by_range(self.drive.target_range)
            if not self.rotation_controller.atSetpoint():
                self.drive.apply_request(lambda: (self.rotation_request
                                                  .with_rotational_rate(rotation_output))).schedule()
            else:
                self.drive.apply_request(lambda: self.brake).schedule()
        else:
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                self.drive.set_clt_target_direction(Rotation2d.fromDegrees(180))
            else:
                self.drive.set_clt_target_direction(Rotation2d.fromDegrees(0))

    def end(self, interrupted: bool):
        self.drive.apply_request(lambda: self.rotation_request).withTimeout(0.01).schedule()
        self.launcher.set_state("off")
