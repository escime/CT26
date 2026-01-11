from commands2 import Command
from wpimath.units import rotationsToDegrees

from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from phoenix6 import swerve
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Rotation2d
from wpilib import DriverStation
from numpy import interp

class Launch(Command):
    def __init__(self, drivetrain: CommandSwerveDrivetrain):
        super().__init__()
        self.drive = drivetrain

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

    def execute(self):
        if self.drive.target_in_view:
            rotation_output = self.rotation_controller.calculate(self.drive.target_yaw, 0)
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
