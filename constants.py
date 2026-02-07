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


class OIConstants:
    kDriverControllerPort = 0
    kOperatorControllerPort = 1


class LEDConstants:
    port = 0
    strip_length = 25


class AutoConstants:
    # Copy these values from TunerConstants.
    _front_left_x_pos: units.meter = inchesToMeters(10.625)
    _front_left_y_pos: units.meter = inchesToMeters(10.625)
    _front_right_x_pos: units.meter = inchesToMeters(10.625)
    _front_right_y_pos: units.meter = inchesToMeters(-10.625)
    _back_left_x_pos: units.meter = inchesToMeters(-10.625)
    _back_left_y_pos: units.meter = inchesToMeters(10.625)
    _back_right_x_pos: units.meter = inchesToMeters(-10.625)
    _back_right_y_pos: units.meter = inchesToMeters(-10.625)

    # Math for auto based on copied units from Tuner.
    _front_left_translation = Translation2d(_front_left_x_pos, _front_left_y_pos)
    _front_right_translation = Translation2d(_front_right_x_pos, _front_right_y_pos)
    _back_left_translation = Translation2d(_back_left_x_pos, _back_left_y_pos)
    _back_right_translation = Translation2d(_back_right_x_pos, _back_right_y_pos)
    kinematics = SwerveDrive4Kinematics(_front_left_translation, _front_right_translation, _back_left_translation,
                                        _back_right_translation)

    drive_base_radius = Translation2d(_front_left_x_pos, _front_left_y_pos).norm()
    speed_at_12_volts: units.meters_per_second = 5.12

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

    range_tof_table = [0, 100] # meters
    time_tof_table = [0, 50] # seconds

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
    state_values = {"off": 0, # In rotations per second
                    "safety": 100 / 60,
                    "standby": 500 / 60,
                    "outpost": 500 / 60,
                    "tower": 2500 / 60,
                    "hub": 2000 / 60,
                    "feed": 500 / 60}
    hood_state_values = {"off": 0,
                         "safety": 0,
                         "standby": 0,
                         "outpost": 1,
                         "tower": 0.5,
                         "hub": 0,
                         "feed": 1}

    # Setup Values -----------------------------------------------------------------------------------------------------
    flywheel_main_can_id = 30
    flywheel_follower_can_id = 31
    stator_current_limit = 120
    supply_current_limit = 60
    gear_ratio = 1
    direction = InvertedValue.CLOCKWISE_POSITIVE
    direction_2 = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    flywheel_rps_threshold = 2

    gp_sensor_port = 1
    launch_time_threshold = 0.5

    hood_can_id = 32
    hood_stator_current_limit = 120
    hood_supply_current_limit = 40
    hood_gear_ratio = 1 # (330 * 24) / (15 * 12)
    hood_direction = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

    # Tuning values ----------------------------------------------------------------------------------------------------
    mm_cruise_velocity = 0 # 15
    mm_acceleration = 0 # 15
    mm_jerk = 0 # 50
    ks = 0.1
    kv = 0 # 0.1
    ka = 0 # 0.4
    kp = 2 # 0.5
    ki = 0
    kd = 0
    torque_feedforward = 3

    hood_mm_cruise_velocity = 10
    hood_mm_acceleration = 10
    hood_mm_jerk = 100
    hood_kg = 0.24
    hood_ks = 0.1
    hood_kv = 0.1
    hood_ka = 0
    hood_kp = 75
    hood_ki = 0
    hood_kd = 0

    # Launcher Table ---------------------------------------------------------------------------------------------------
    # range in meters, hood angle in 0-1, shooter speed in RPM
    launcher_table = [
        [0, 0, 2000 / 60],
        [15, 0.2, 4000 / 60]
    ]

class IntakeConstants:
    # [Intake speed, Deploy power]
    state_values = {"stow": [0, -60],
                    "intake": [12, 60],
                    "deployed": [0, 10],
                    "outpost": [-12, 10],
                    "launching": [6, -45],
                    "jam_clear": [-12, 0]
                    }

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
    intake_deploy_peak_forward_current = 60
    intake_deploy_peak_reverse_current = -60

class HopperConstants:
    # spindexer right (volts), spindexer left (volts), feeder (rotations/sec)
    state_values = {
        "off": [0, 0, 0],
        "launching": [4, 4, 2000 / 60],
        "intaking": [-2, -2, 0],
        "jam_clear": [-8, -8, -500 / 60]
    }

    right_indexer_can_id = 50
    left_indexer_can_id = 51
    stator_current_limit = 120
    supply_current_limit = 40
    indexer_gear_ratio = 4
    direction = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    direction_2 = InvertedValue.CLOCKWISE_POSITIVE

    feeder_direction = InvertedValue.CLOCKWISE_POSITIVE
    feeder_can_id = 52
    feeder_gear_ratio = 1

    kp = 0.2
    ki = 0
    kd = 0


class ClimberConstants:
    # voltage, position
    state_values = {
        "stow": [-12, 0],
        "deployed": [12, 0],
        "climb": [-12, 1]
    }

    climber_can_id = 60
    ranger_front_can_id = 61
    ranger_back_can_id = 62
    servo_port = 1

    threshold_range_short = inchesToMeters(20)
    threshold_range_long = 3

    stator_current_limit = 120
    supply_current_limit = 40
    gear_ratio = 1
    direction = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

    deployed_position = 1
    stowed_position = 0
    climbed_position = 0.5