"""
The constants module is a convenience place for teams to hold robot-wide
numerical or boolean constants. Don't use this for any other purpose!
"""
from wpimath.units import inchesToMeters, degreesToRadians, feetToMeters, lbsToKilograms, \
    rotationsPerMinuteToRadiansPerSecond
from phoenix6 import units
from wpimath.geometry import Translation2d, Translation3d, Transform3d, Rotation3d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from photonlibpy import photonCamera, photonPoseEstimator
from phoenix6.signals import InvertedValue
from math import pi, cos


class OIConstants:
    kDriverControllerPort = 0
    kOperatorControllerPort = 1


class LEDConstants:
    port = 5
    strip_length = 25


class AutoConstants:
    # Copy these values from TunerConstants.
    _front_left_x_pos: units.meter = inchesToMeters(9.625)
    _front_left_y_pos: units.meter = inchesToMeters(9.625)
    _front_right_x_pos: units.meter = inchesToMeters(9.625)
    _front_right_y_pos: units.meter = inchesToMeters(-9.625)
    _back_left_x_pos: units.meter = inchesToMeters(-9.625)
    _back_left_y_pos: units.meter = inchesToMeters(9.625)
    _back_right_x_pos: units.meter = inchesToMeters(-9.625)
    _back_right_y_pos: units.meter = inchesToMeters(-9.625)

    # Math for auto based on copied units from Tuner.
    _front_left_translation = Translation2d(_front_left_x_pos, _front_left_y_pos)
    _front_right_translation = Translation2d(_front_right_x_pos, _front_right_y_pos)
    _back_left_translation = Translation2d(_back_left_x_pos, _back_left_y_pos)
    _back_right_translation = Translation2d(_back_right_x_pos, _back_right_y_pos)
    kinematics = SwerveDrive4Kinematics(_front_left_translation, _front_right_translation, _back_left_translation,
                                        _back_right_translation)

    drive_base_radius = Translation2d(_front_left_x_pos, _front_left_y_pos).norm()
    speed_at_12_volts: units.meters_per_second = 4.73

    # PID Constants for PathPlanner
    x_pid = [5, 0, 0]
    y_pid = [5, 0, 0]

class VisionConstants:
    # AprilTag Information ---------------------------------------------------------------------------------------------
    april_tag_field_layout = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltAndyMark)
    default_tags = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
                    17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32]
    target_height = inchesToMeters(44.25)

    border_tags = [13, 14, 15, 16, 29, 30, 31, 32]
    neutral_zone_tags = [1, 2, 3, 4, 5, 6, 17, 18, 19, 20, 21, 22, 24, 2, 5, 27]
    red_alliance_tags = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
    blue_alliance_tags = [17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32]

    red_hub_center = [11.915, 4.035]
    blue_hub_center = [4.630, 4.035]

    tag_names = {
        "red": {
            "hub":
                {
                    "center_audience": 2,
                    "offset_audience": 11,
                    "offset_neutral": 3,
                    "center_neutral": 4,
                    "center_scoring_table": 5,
                    "offset_scoring_table": 8,
                    "center_tower": 10,
                    "offset_tower": 9
                },
            "trench":
                {
                    "audience_neutral": 1,
                    "audience_tower": 12,
                    "scoring_table_neutral": 6,
                    "scoring_table_tower": 7
                },
            "outpost":
                {
                    "center": 13,
                    "offset": 14
                },
            "tower":
                {
                    "center": 15,
                    "offset": 16
                }
        },
        "blue": {
            "hub":
                {
                    "center_audience": 21,
                    "offset_audience": 24,
                    "offset_neutral": 19,
                    "center_neutral": 20,
                    "center_scoring_table": 18,
                    "offset_scoring_table": 27,
                    "center_tower": 26,
                    "offset_tower": 25
                },
            "trench":
                {
                    "audience_neutral": 22,
                    "audience_tower": 23,
                    "scoring_table_neutral": 17,
                    "scoring_table_tower": 28
                },
            "outpost":
                {
                    "center": 29,
                    "offset": 30
                },
            "tower":
                {
                    "center": 31,
                    "offset": 32
                }
        }
    }

    # Camera 1 Information ---------------------------------------------------------------------------------------------
    cam_1_name = "launcher"
    robot_to_cam1 = Transform3d(Translation3d(inchesToMeters(-8.827717),
                                              inchesToMeters(-10.1875),
                                              inchesToMeters(20.666455)),
                                Rotation3d(degreesToRadians(0),
                                           degreesToRadians(-25),
                                           degreesToRadians(0)))

    cam1 = photonCamera.PhotonCamera(cam_1_name)

    # cam1_pose = (
    #     photonPoseEstimator.PhotonPoseEstimator(april_tag_field_layout,
    #                                             photonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    #                                             cam1,
    #                                             robot_to_cam1))

    cam1_pose = (
        photonPoseEstimator.PhotonPoseEstimator(april_tag_field_layout,
                                                robot_to_cam1)
    )

    # Camera 2 Information ---------------------------------------------------------------------------------------------
    cam_2_name = "back_right"
    robot_to_cam2 = Transform3d(Translation3d(inchesToMeters(-11.124557),
                                              inchesToMeters(-10.340702),
                                              inchesToMeters(9.208135)),
                                Rotation3d(degreesToRadians(0),
                                           degreesToRadians(-20),
                                           degreesToRadians(30 + 180)))

    cam2 = photonCamera.PhotonCamera(cam_2_name)

    # cam2_pose = (
    #     photonPoseEstimator.PhotonPoseEstimator(april_tag_field_layout,
    #                                             photonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    #                                             cam2,
    #                                             robot_to_cam2))

    cam2_pose = (
        photonPoseEstimator.PhotonPoseEstimator(april_tag_field_layout,
                                                robot_to_cam2)
    )

    # Camera 3 Information ---------------------------------------------------------------------------------------------
    cam_3_name = "back_left"
    robot_to_cam3 = Transform3d(Translation3d(inchesToMeters(-11.124557),
                                              inchesToMeters(10.340702),
                                              inchesToMeters(9.208135)),
                                Rotation3d(degreesToRadians(0),
                                           degreesToRadians(-20),
                                           degreesToRadians(-30 + 180)))

    cam3 = photonCamera.PhotonCamera(cam_3_name)

    # cam3_pose = (
    #     photonPoseEstimator.PhotonPoseEstimator(april_tag_field_layout,
    #                                             photonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    #                                             cam3,
    #                                             robot_to_cam3))

    cam3_pose = (
        photonPoseEstimator.PhotonPoseEstimator(april_tag_field_layout,
                                                robot_to_cam3)
    )

    # Camera Lists -----------------------------------------------------------------------------------------------------
    robot_cameras_3d = [cam1, cam2, cam3]
    robot_cameras_poses_3d = [cam1_pose, cam2_pose, cam3_pose]
    robot_cameras_2d = [cam1]
    robot_cameras_2d_height = robot_to_cam1.z
    robot_cameras_2d_angle = abs(robot_to_cam1.rotation().y_degrees)

class LauncherConstants:
    # Setup Values -----------------------------------------------------------------------------------------------------
    flywheel_main_can_id = 30
    flywheel_follower_can_id = 31
    stator_current_limit = 120
    supply_current_limit = 60
    gear_ratio = 1
    direction = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    flywheel_rps_threshold = 5

    gp_sensor_port = 1
    launch_time_threshold = 0.5

    hood_can_id = 32
    hood_stator_current_limit = 120
    hood_supply_current_limit = 40
    hood_gear_ratio = 0.1
    hood_direction = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

    # Tuning values ----------------------------------------------------------------------------------------------------
    mm_cruise_velocity = 100
    mm_acceleration = 30
    mm_jerk = 100
    ks = 0.25
    kv = 0.09
    ka = 1.56
    kp = 1
    ki = 0
    kd = 0

    hood_mm_cruise_velocity = 2.5
    hood_mm_acceleration = 1.25
    hood_mm_jerk = 100
    hood_kg = 0.01
    hood_ks = 0.25
    hood_kv = 1.08
    hood_ka = 0.0
    hood_kp = 1
    hood_ki = 0
    hood_kd = 0

    # Launcher Table ---------------------------------------------------------------------------------------------------
    # range in meters, hood angle in 0-1, shooter speed in RPM
    launcher_table = [
        [0, 0, 2000 / 60],
        [15, 1, 2500 / 60]
    ]

class IntakeConstants:
    intake_leader_can_id = 40
    intake_follower_can_id = 41
    stator_current_limit = 120
    supply_current_limit = 40
    gear_ratio = 1
    direction = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

    intake_deploy_can_id = 42
    max_duty_cycle = 0.75
    intake_deploy_stator_current_limit = 120
    intake_deploy_supply_current_limit = 40
    intake_deploy_gear_ratio = 20
    intake_deploy_peak_forward_current = 30
    intake_deploy_peak_reverse_current = 30

    # [Intake speed, Deploy power]
    state_values = {"stow": [0, -20],
                    "intake": [12, 30],
                    "deployed": [0, 5],
                    "outpost": [-12, 10]
                    }
