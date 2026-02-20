from commands2 import Subsystem, sysid, Command
from phoenix6 import SignalLogger

from phoenix6.hardware import TalonFX
from phoenix6.controls import VoltageOut, MotionMagicVelocityVoltage, Follower, MotionMagicVoltage, VelocityTorqueCurrentFOC
from phoenix6.configs import TalonFXConfiguration
from phoenix6.status_code import StatusCode
from phoenix6.signals import MotorAlignmentValue, NeutralModeValue
from phoenix6.utils import get_current_time_seconds, is_simulation
from phoenix6.canbus import CANBus

from wpilib import DigitalInput
from wpilib.simulation import FlywheelSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.units import radiansToRotations
from ntcore import NetworkTableInstance
from wpilib.sysid import SysIdRoutineLog

from math import pi, degrees
from numpy import interp
from constants import LauncherConstants


class LauncherSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self._debug_mode = False
        self._last_sim_time = get_current_time_seconds()
        self.state_values = LauncherConstants.state_values
        self.hood_state_values = LauncherConstants.hood_state_values
        self.state = "off"
        self.auto_velocity = 1500 / 60
        self.auto_hood_position = 0

        self._inst = NetworkTableInstance.getDefault()
        self._launcher_table = self._inst.getTable("Launcher")

        # Flywheel Setup -----------------------------------------------------------------------------------------------
        self.flywheel = TalonFX(LauncherConstants.flywheel_main_can_id, CANBus("rio"))
        self.flywheel_follower = TalonFX(LauncherConstants.flywheel_follower_can_id, CANBus("rio"))

        self.flywheel_mm = MotionMagicVelocityVoltage(0, enable_foc=True)

        self.flywheel_tvfoc = VelocityTorqueCurrentFOC(velocity=0,
                                                       feed_forward=3,
                                                       slot=0)

        flywheel_configs = TalonFXConfiguration()

        flywheel_configs.current_limits.stator_current_limit = LauncherConstants.stator_current_limit
        flywheel_configs.current_limits.stator_current_limit_enable = True
        flywheel_configs.current_limits.supply_current_limit = LauncherConstants.supply_current_limit
        flywheel_configs.current_limits.supply_current_limit_enable = True
        flywheel_configs.feedback.sensor_to_mechanism_ratio = LauncherConstants.gear_ratio
        flywheel_configs.motor_output.inverted = LauncherConstants.direction

        flywheel_mm_configs = flywheel_configs.motion_magic
        flywheel_mm_configs.motion_magic_cruise_velocity = LauncherConstants.mm_cruise_velocity
        flywheel_mm_configs.motion_magic_acceleration = LauncherConstants.mm_acceleration
        flywheel_mm_configs.motion_magic_jerk = LauncherConstants.mm_jerk

        flywheel_slot0_configs = flywheel_configs.slot0
        flywheel_slot0_configs.with_k_s(LauncherConstants.ks)
        flywheel_slot0_configs.with_k_v(LauncherConstants.kv)
        flywheel_slot0_configs.with_k_a(LauncherConstants.ka)
        flywheel_slot0_configs.with_k_p(LauncherConstants.kp)
        flywheel_slot0_configs.with_k_i(LauncherConstants.ki)
        flywheel_slot0_configs.with_k_d(LauncherConstants.kd)

        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.flywheel.configurator.apply(flywheel_configs)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        flywheel_configs.motor_output.inverted = LauncherConstants.direction_2
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.flywheel_follower.configurator.apply(flywheel_configs)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        # self.flywheel_follower.set_control(Follower(LauncherConstants.flywheel_main_can_id, MotorAlignmentValue.OPPOSED))

        self.flywheel_sim = self.flywheel.sim_state
        self.flywheel_follower_sim = self.flywheel_follower.sim_state

        self.wheel_sim = FlywheelSim(
            LinearSystemId.flywheelSystem(DCMotor.krakenX60FOC(2), 0.005, LauncherConstants.gear_ratio),
            DCMotor.krakenX60FOC(1),
            [0.0]
        )

        self.wheel_follower_sim = FlywheelSim(
            LinearSystemId.flywheelSystem(DCMotor.krakenX60FOC(2), 0.005, LauncherConstants.gear_ratio),
            DCMotor.krakenX60FOC(1),
            [0.0]
        )

        self.flywheel_volts = VoltageOut(0, False)

        # Hood Setup ---------------------------------------------------------------------------------------------------
        self.hood = TalonFX(LauncherConstants.hood_can_id, CANBus("rio"))
        hood_config = TalonFXConfiguration()
        self.hood_mm = MotionMagicVoltage(0, True)

        hood_config.current_limits.supply_current_limit = LauncherConstants.hood_supply_current_limit
        hood_config.current_limits.supply_current_limit_enable = True
        hood_config.current_limits.stator_current_limit = LauncherConstants.hood_stator_current_limit
        hood_config.current_limits.stator_current_limit_enable = True
        hood_config.feedback.sensor_to_mechanism_ratio = LauncherConstants.hood_gear_ratio
        hood_config.motor_output.neutral_mode = NeutralModeValue.BRAKE


        hood_mm_config = hood_config.motion_magic
        hood_mm_config.motion_magic_cruise_velocity = LauncherConstants.hood_mm_cruise_velocity
        hood_mm_config.motion_magic_acceleration = LauncherConstants.hood_mm_acceleration
        hood_mm_config.motion_magic_jerk = LauncherConstants.hood_mm_jerk

        hood_slot0_config = hood_config.slot0
        hood_slot0_config.k_g = LauncherConstants.hood_kg
        hood_slot0_config.k_s = LauncherConstants.hood_ks
        hood_slot0_config.k_v = LauncherConstants.hood_kv
        hood_slot0_config.k_a = LauncherConstants.hood_ka
        hood_slot0_config.k_p = LauncherConstants.hood_kp
        hood_slot0_config.k_i = LauncherConstants.hood_ki
        hood_slot0_config.k_d = LauncherConstants.hood_kd

        hood_torque_config = hood_config.torque_current
        hood_torque_config.with_peak_forward_torque_current(10)
        hood_torque_config.with_peak_reverse_torque_current(10)

        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.hood.configurator.apply(hood_config)
            if status.is_ok():
                print("Configuration applied.")
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        self.hood.set_position(0)

        # Other Setup --------------------------------------------------------------------------------------------------
        self.gp_sensor = DigitalInput(LauncherConstants.gp_sensor_port)
        self.last_launch_time = get_current_time_seconds()
        self.last_time = get_current_time_seconds()
        self.setpoint_enabled_time = get_current_time_seconds()

        # SYSID Setup --------------------------------------------------------------------------------------------------
        self.sys_id_control = VoltageOut(0)

        self._sysid_routine_leader = sysid.SysIdRoutine(
            sysid.SysIdRoutine.Config(
                stepVoltage=4.0,
                recordState=lambda state: SignalLogger.write_string(
                    "SysId_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            sysid.SysIdRoutine.Mechanism(
                lambda volts: self.flywheel.set_control(self.sys_id_control.with_output(volts)),
                lambda log: None,
                self
            )
        )
        self._sysid_routine_follower = sysid.SysIdRoutine(
            sysid.SysIdRoutine.Config(
                stepVoltage=4.0,
                recordState=lambda state: SignalLogger.write_string(
                    "SysId_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            sysid.SysIdRoutine.Mechanism(
                lambda volts: self.flywheel_follower.set_control(self.sys_id_control.with_output(volts)),
                lambda log: None,
                self
            )
        )
        self.setName("Launcher")

        self._launcher_table.putNumber("Flywheel MM Cruise Velocity", LauncherConstants.mm_cruise_velocity)
        self._launcher_table.putNumber("Flywheel MM Acceleration", LauncherConstants.mm_acceleration)
        self._launcher_table.putNumber("Flywheel MM Jerk", LauncherConstants.mm_jerk)
        self._launcher_table.putNumber("Flywheel kS", LauncherConstants.ks)
        self._launcher_table.putNumber("Flywheel kV", LauncherConstants.kv)
        self._launcher_table.putNumber("Flywheel kA", LauncherConstants.ka)
        self._launcher_table.putNumber("Flywheel kP", LauncherConstants.kp)
        self._launcher_table.putNumber("Flywheel kI", LauncherConstants.ki)
        self._launcher_table.putNumber("Flywheel kD", LauncherConstants.kd)
        self._launcher_table.putNumber("Torque Feedforward", LauncherConstants.torque_feedforward)

        self._launcher_table.putNumber("Hood MM Cruise Velocity", LauncherConstants.hood_mm_cruise_velocity)
        self._launcher_table.putNumber("Hood MM Acceleration", LauncherConstants.hood_mm_acceleration)
        self._launcher_table.putNumber("Hood MM Jerk", LauncherConstants.hood_mm_jerk)
        self._launcher_table.putNumber("Hood kG", LauncherConstants.hood_kg)
        self._launcher_table.putNumber("Hood kS", LauncherConstants.hood_ks)
        self._launcher_table.putNumber("Hood kV", LauncherConstants.hood_kv)
        self._launcher_table.putNumber("Hood kA", LauncherConstants.hood_ka)
        self._launcher_table.putNumber("Hood kP", LauncherConstants.hood_kp)
        self._launcher_table.putNumber("Hood kI", LauncherConstants.hood_ki)
        self._launcher_table.putNumber("Hood kD", LauncherConstants.hood_kd)

        self._launcher_table.putNumber("Testing Launcher Speed", 2000)
        self._launcher_table.putNumber("Testing Launcher Hood Angle", 0)

    def live_reconfigure(self) -> None:
        flywheel_configs = TalonFXConfiguration()
        flywheel_configs.current_limits.stator_current_limit = LauncherConstants.stator_current_limit
        flywheel_configs.current_limits.stator_current_limit_enable = True
        flywheel_configs.current_limits.supply_current_limit = LauncherConstants.supply_current_limit
        flywheel_configs.current_limits.supply_current_limit_enable = True
        flywheel_configs.feedback.sensor_to_mechanism_ratio = LauncherConstants.gear_ratio
        flywheel_configs.motor_output.inverted = LauncherConstants.direction

        flywheel_mm_configs = flywheel_configs.motion_magic
        flywheel_mm_configs.motion_magic_cruise_velocity = (
            self._launcher_table.getNumber("Flywheel MM Cruise Velocity", LauncherConstants.mm_cruise_velocity))
        flywheel_mm_configs.motion_magic_acceleration = (
            self._launcher_table.getNumber("Flywheel MM Acceleration", LauncherConstants.mm_acceleration))
        flywheel_mm_configs.motion_magic_jerk = (
            self._launcher_table.getNumber("Flywheel MM Jerk", LauncherConstants.mm_jerk))

        flywheel_slot0_configs = flywheel_configs.slot0
        flywheel_slot0_configs.with_k_s(self._launcher_table.getNumber("Flywheel kS", LauncherConstants.ks))
        flywheel_slot0_configs.with_k_v(self._launcher_table.getNumber("Flywheel kV", LauncherConstants.kv))
        flywheel_slot0_configs.with_k_a(self._launcher_table.getNumber("Flywheel kA", LauncherConstants.ka))
        flywheel_slot0_configs.with_k_p(self._launcher_table.getNumber("Flywheel kP", LauncherConstants.kp))
        flywheel_slot0_configs.with_k_i(self._launcher_table.getNumber("Flywheel kI", LauncherConstants.ki))
        flywheel_slot0_configs.with_k_d(self._launcher_table.getNumber("Flywheel kD", LauncherConstants.kd))
        self.flywheel_tvfoc.feed_forward = self._launcher_table.getNumber("Torque Feedforward", LauncherConstants.torque_feedforward)

        hood_config = TalonFXConfiguration()
        hood_config.current_limits.supply_current_limit = LauncherConstants.hood_supply_current_limit
        hood_config.current_limits.supply_current_limit_enable = True
        hood_config.current_limits.stator_current_limit = LauncherConstants.hood_stator_current_limit
        hood_config.current_limits.stator_current_limit_enable = True
        hood_config.feedback.sensor_to_mechanism_ratio = LauncherConstants.hood_gear_ratio
        hood_config.motor_output.neutral_mode = NeutralModeValue.BRAKE

        hood_mm_config = hood_config.motion_magic
        hood_mm_config.motion_magic_cruise_velocity = (
            self._launcher_table.getNumber("Hood MM Cruise Velocity", LauncherConstants.hood_mm_cruise_velocity))
        hood_mm_config.motion_magic_acceleration = (
            self._launcher_table.getNumber("Hood MM Acceleration", LauncherConstants.hood_mm_acceleration))
        hood_mm_config.motion_magic_jerk = (
            self._launcher_table.getNumber("Hood MM Jerk", LauncherConstants.hood_mm_jerk))

        hood_slot0_config = hood_config.slot0
        hood_slot0_config.k_g = self._launcher_table.getNumber("Hood kG", LauncherConstants.hood_kg)
        hood_slot0_config.k_s = self._launcher_table.getNumber("Hood kS", LauncherConstants.hood_ks)
        hood_slot0_config.k_v = self._launcher_table.getNumber("Hood kV", LauncherConstants.hood_kv)
        hood_slot0_config.k_a = self._launcher_table.getNumber("Hood kA", LauncherConstants.hood_ka)
        hood_slot0_config.k_p = self._launcher_table.getNumber("Hood kP", LauncherConstants.hood_kp)
        hood_slot0_config.k_i = self._launcher_table.getNumber("Hood kI", LauncherConstants.hood_ki)
        hood_slot0_config.k_d = self._launcher_table.getNumber("Hood kD", LauncherConstants.hood_kd)

        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        status3: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.flywheel.configurator.apply(flywheel_configs)
            status3 = self.hood.configurator.apply(hood_config)
            if status.is_ok() and status3.is_ok():
                print("Configuration applied.")
                break
        if not status.is_ok() or not status3.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        flywheel_configs.motor_output.inverted = LauncherConstants.direction_2
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.flywheel_follower.configurator.apply(flywheel_configs)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

    def set_state(self, state: str) -> None:
        self.setpoint_enabled_time = get_current_time_seconds()
        self.state = state
        if state == "auto":
            self.flywheel.set_control(self.flywheel_tvfoc.with_velocity(self.auto_velocity).with_slot(0))
            self.flywheel_follower.set_control(self.flywheel_tvfoc.with_velocity(self.auto_velocity).with_slot(0))
            self.hood.set_control(self.hood_mm.with_position(self.auto_hood_position).with_slot(0))
        elif state == "testing":
            self.flywheel.set_control(self.flywheel_tvfoc.with_velocity(self._launcher_table.getNumber("Testing Launcher Speed", 2000) / 60).with_slot(0))
            self.flywheel_follower.set_control(self.flywheel_tvfoc.with_velocity(self._launcher_table.getNumber("Testing Launcher Speed", 2000) / 60).with_slot(0))
            self.hood.set_control(self.hood_mm.with_position(self._launcher_table.getNumber("Testing Launcher Hood Angle", 0)).with_slot(0))
        elif state == "off":
            self.flywheel.set_control(self.flywheel_volts.with_output(0))
            self.flywheel_follower.set_control(self.flywheel_volts.with_output(0))
            self.hood.set_control(self.hood_mm.with_position(0).with_slot(0))
        else:
            self.flywheel.set_control(self.flywheel_tvfoc.with_velocity(self.state_values[state]).with_slot(0))
            self.flywheel_follower.set_control(self.flywheel_tvfoc.with_velocity(self.state_values[state]).with_slot(0))
            self.hood.set_control(self.hood_mm.with_position(self.hood_state_values[state]).with_slot(0))

    def set_flywheel_auto_default_velocity(self, velocity: float) -> None:
        self.auto_velocity = velocity

    def set_debug_mode(self, on: bool) -> None:
        self._debug_mode = on

    def set_target_by_range(self, distance_to_goal: float) -> None:
        self.state = "auto"
        distance_array = []
        hood_angle_array = []
        launcher_speed_array = []
        for x in LauncherConstants.launcher_table:
            distance_array.append(x[0])
            hood_angle_array.append(x[1])
            launcher_speed_array.append(x[2])

        self.hood.set_control(self.hood_mm.with_position(interp(distance_to_goal, distance_array, hood_angle_array)))
        self.flywheel.set_control(self.flywheel_tvfoc.with_velocity(interp(distance_to_goal, distance_array, launcher_speed_array)))
        self.flywheel_follower.set_control(self.flywheel_tvfoc.with_velocity(interp(distance_to_goal, distance_array, launcher_speed_array)))
        self.auto_velocity = interp(distance_to_goal, distance_array, launcher_speed_array)
        self.auto_hood_position = interp(distance_to_goal, distance_array, hood_angle_array)

    def get_last_launch_recorded(self):
        return self.last_launch_time

    def record_launch_time(self):
        if self.get_sensor_on():
            self.last_launch_time = get_current_time_seconds()
        else:
            pass

    def get_still_launching(self):
        if get_current_time_seconds() - self.get_last_launch_recorded() > LauncherConstants.launch_time_threshold:
            return True
        else:
            return False

    def get_state(self) -> str:
        return self.state

    def get_sensor_on(self) -> bool:
        return not self.gp_sensor.get()

    def get_velocity(self) -> [float, float]:
        return [self.flywheel.get_velocity(True).value_as_double, self.flywheel_follower.get_velocity(True).value_as_double]

    def get_at_target(self) -> bool:
        if self.state == "off":
            return True
        elif self.state == "auto":
            if self.auto_velocity - LauncherConstants.flywheel_rps_threshold < self.get_velocity()[0] <= self.auto_velocity + LauncherConstants.flywheel_rps_threshold:
                return True
            else:
                return False
        elif self.state == "testing":
            if (self._launcher_table.getNumber("Testing Launcher Speed", 2000) / 60) - LauncherConstants.flywheel_rps_threshold < self.get_velocity()[0] <= (self._launcher_table.getNumber("Testing Launcher Speed", 2000) / 60) + LauncherConstants.flywheel_rps_threshold:
                return True
            else:
                return False
        else:
            if self.state_values[self.state] - LauncherConstants.flywheel_rps_threshold < self.get_velocity()[0] <= self.state_values[self.state] + LauncherConstants.flywheel_rps_threshold:
                return True
            else:
                return False

    def set_voltage_direct(self, output: float):
        self.flywheel.set_control(self.flywheel_volts.with_output(output))

    def sys_id_quasistatic_leader(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self._sysid_routine_leader.quasistatic(direction)

    def sys_id_dynamic_leader(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self._sysid_routine_leader.dynamic(direction)

    def sys_id_quasistatic_follower(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self._sysid_routine_follower.quasistatic(direction)

    def sys_id_dynamic_follower(self, direction: sysid.SysIdRoutine.Direction) -> Command:
        return self._sysid_routine_follower.dynamic(direction)

    def update_sim(self):
        current_time = get_current_time_seconds()
        dt = current_time - self.last_time
        self.last_time = current_time
        self.wheel_sim.setInput(0, self.flywheel_sim.motor_voltage)
        self.wheel_follower_sim.setInput(0, self.flywheel_follower_sim.motor_voltage)
        self.wheel_sim.update(dt)
        self.wheel_follower_sim.update(dt)
        self.flywheel_follower_sim.set_rotor_velocity(radiansToRotations(self.wheel_follower_sim.getAngularVelocity() * LauncherConstants.gear_ratio))
        self.flywheel_sim.set_rotor_velocity(radiansToRotations(self.wheel_sim.getAngularVelocity() * LauncherConstants.gear_ratio))

    def periodic(self) -> None:
        if is_simulation():
            self.update_sim()

        self.record_launch_time()

        if self._debug_mode:
            self._launcher_table.putNumber("Left Flywheel Velocity", self.get_velocity()[0])
            self._launcher_table.putNumber("Right Flywheel Velocity", self.get_velocity()[1])
            self._launcher_table.putNumber("Time Since Setpoint Activated", get_current_time_seconds() - self.setpoint_enabled_time)
            self._launcher_table.putNumber("Flywheel Auto Target", self.auto_velocity)
            self._launcher_table.putNumber("Hood Angle", self.hood.get_position().value_as_double)
            self._launcher_table.putNumber("Hood Auto Target", self.auto_hood_position)
            self._launcher_table.putBoolean("Launcher Sensor", self.get_sensor_on())

        self._launcher_table.putBoolean("Flywheel at Speed", self.get_at_target())
