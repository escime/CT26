from commands2 import Command

from subsystems.climbersubsystem import ClimberSubsystem
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from generated.tuner_constants import TunerConstants

from phoenix6 import swerve
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from wpilib import DriverStation
from wpimath.units import inchesToMeters

from math import pi

from constants import ClimberConstants


class AutoClimb(Command):

    def __init__(self, drive: CommandSwerveDrivetrain, climber: ClimberSubsystem):
        super().__init__()
        self._drive = drive
        self._climber = climber

        self.drive_request = (swerve.requests.FieldCentricFacingAngle()
                              .with_drive_request_type(swerve.SwerveModule.DriveRequestType.VELOCITY))

        self.drive_request.heading_controller.setPID(5, 0, 0)
        self.drive_request.heading_controller.enableContinuousInput(0, -2 * pi)
        self.drive_request.heading_controller.setTolerance(0.05)

        self._blue_alliance = True

    def initialize(self):
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            self._blue_alliance = False
        else:
            self._blue_alliance = True

        self._climber.set_state("deployed")

    def execute(self):
        print(self._climber.get_range_front())
        if self._climber.get_range_front() >= ClimberConstants.threshold_range_short + 0.01:
            if self._blue_alliance:
                x_output = 0.1
                direction = 180
            else:
                x_output = -0.1
                direction = 0

            (self._drive.apply_request(lambda: self.drive_request
                                       .with_velocity_x(x_output * TunerConstants.speed_at_12_volts)
                                       .with_velocity_y(0)
                                       .with_target_direction(Rotation2d.fromDegrees(direction))).schedule())
        elif self._climber.get_range_front() < ClimberConstants.threshold_range_short - 0.01:
            if self._blue_alliance:
                x_output = -0.1
                direction = 180
            else:
                x_output = 0.1
                direction = 0

            (self._drive.apply_request(lambda: self.drive_request
                                       .with_velocity_x(x_output * TunerConstants.speed_at_12_volts)
                                       .with_velocity_y(0)
                                       .with_target_direction(Rotation2d.fromDegrees(direction))).schedule())

        else:
            if self._climber.get_range_back() < ClimberConstants.threshold_range_long:
                if self._blue_alliance:
                    y_output = -0.1
                    direction = 180
                else:
                    y_output = 0.1
                    direction = 0

                (self._drive.apply_request(lambda: self.drive_request
                                           .with_velocity_x(0)
                                           .with_velocity_y(y_output * TunerConstants.speed_at_12_volts)
                                           .with_target_direction(Rotation2d.fromDegrees(direction))).schedule())
            else:
                self._climber.set_state("climb")

    def isFinished(self) -> bool:
        if self._climber.get_state() == "climb":
            return True
        else:
            return False

    def end(self, interrupted: bool):
        (self._drive.apply_request(lambda: self.drive_request
                                  .with_velocity_x(0)
                                  .with_velocity_y(0)
                                  .with_target_direction(self._drive.get_pose().rotation()))
         .withTimeout(0.02).schedule())