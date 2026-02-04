from commands2 import Subsystem

from phoenix6.hardware import TalonFX
from phoenix6.controls import VoltageOut, Follower, TorqueCurrentFOC
from phoenix6.configs import TalonFXConfiguration
from phoenix6.status_code import StatusCode
from phoenix6.signals import MotorAlignmentValue
from phoenix6.utils import get_current_time_seconds, is_simulation
from phoenix6.canbus import CANBus

from wpimath.system.plant import DCMotor
from wpilib.simulation import SingleJointedArmSim
from wpimath.units import radiansToRotations, inchesToMeters, lbsToKilograms
from ntcore import NetworkTableInstance

from math import pi
from constants import IntakeConstants


class IntakeSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self._last_sim_time = get_current_time_seconds()
        self.state_values = IntakeConstants.state_values
        self.state = "stow"

        self._inst = NetworkTableInstance.getDefault()
        self._intake_table = self._inst.getTable("Intake")

        # Intake Setup -----------------------------------------------------------------------------------------------
        self.intake_leader = TalonFX(IntakeConstants.intake_leader_can_id, CANBus("rio"))
        # self.intake_follower = TalonFX(IntakeConstants.intake_follower_can_id, CANBus("rio"))

        self.intake_volts = VoltageOut(0, True)

        intake_configs = TalonFXConfiguration()

        intake_configs.current_limits.stator_current_limit = IntakeConstants.stator_current_limit
        intake_configs.current_limits.stator_current_limit_enable = True
        intake_configs.current_limits.supply_current_limit = IntakeConstants.supply_current_limit
        intake_configs.current_limits.supply_current_limit_enable = True
        intake_configs.feedback.sensor_to_mechanism_ratio = IntakeConstants.gear_ratio
        intake_configs.motor_output.inverted = IntakeConstants.direction

        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        # status_2: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.intake_leader.configurator.apply(intake_configs)
            # status_2 = self.intake_follower.configurator.apply(intake_configs)
            # if status.is_ok() and status_2.is_ok():
            if status.is_ok():
                break
        # if not status.is_ok() or not status_2.is_ok():
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        # self.intake_follower.set_control(Follower(IntakeConstants.intake_leader_can_id, MotorAlignmentValue.OPPOSED))

        # Intake Deploy Setup ------------------------------------------------------------------------------------------
        self.intake_deploy = TalonFX(IntakeConstants.intake_deploy_can_id, CANBus("rio"))
        intake_deploy_config = TalonFXConfiguration()
        self.intake_deploy_tfoc = TorqueCurrentFOC(0,
                                                   IntakeConstants.max_duty_cycle,
                                                   2,
                                                   True
                                                   )

        intake_deploy_config.motor_output.neutral_mode = intake_deploy_config.motor_output.neutral_mode.COAST
        intake_deploy_config.motor_output.inverted = intake_deploy_config.motor_output.inverted.COUNTER_CLOCKWISE_POSITIVE

        intake_deploy_config.current_limits.supply_current_limit = IntakeConstants.intake_deploy_stator_current_limit
        intake_deploy_config.current_limits.supply_current_limit_enable = True
        intake_deploy_config.current_limits.stator_current_limit = IntakeConstants.intake_deploy_stator_current_limit
        intake_deploy_config.current_limits.stator_current_limit_enable = True
        intake_deploy_config.feedback.sensor_to_mechanism_ratio = IntakeConstants.intake_deploy_gear_ratio

        intake_deploy_torque_config = intake_deploy_config.torque_current
        # intake_deploy_torque_config.with_peak_forward_torque_current(IntakeConstants.intake_deploy_peak_forward_current)
        # intake_deploy_torque_config.with_peak_reverse_torque_current(IntakeConstants.intake_deploy_peak_reverse_current)
        intake_deploy_torque_config.with_torque_neutral_deadband(2)

        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.intake_deploy.configurator.apply(intake_deploy_config)
            if status.is_ok():
                print("Configuration applied.")
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        self.intake_deploy.set_position(0)

        self.intake_deploy_sim = self.intake_deploy.sim_state
        self.arm_sim = SingleJointedArmSim(
            DCMotor.krakenX60(1),
            IntakeConstants.intake_deploy_gear_ratio,
            SingleJointedArmSim.estimateMOI(inchesToMeters(12), lbsToKilograms(5)),
            inchesToMeters(3),
            -0.1,
            pi + 0.1,
            True,
            1
        )

        # Other Setup --------------------------------------------------------------------------------------------------
        self.last_time = get_current_time_seconds()

    def set_state(self, state: str) -> None:
        self.state = state
        self.intake_leader.set_control(self.intake_volts.with_output(self.state_values[state][0]))
        self.intake_deploy.set_control(self.intake_deploy_tfoc.with_output(self.state_values[state][1]))

    def get_state(self) -> str:
        return self.state

    def update_sim(self):
        current_time = get_current_time_seconds()
        dt = current_time - self.last_time
        self.last_time = current_time
        self.arm_sim.setInput(0, self.intake_deploy_sim.motor_voltage)
        self.arm_sim.update(dt)
        self.intake_deploy_sim.set_raw_rotor_position(radiansToRotations(self.arm_sim.getAngle() * IntakeConstants.intake_deploy_gear_ratio))
        self.intake_deploy_sim.set_rotor_velocity(radiansToRotations(self.arm_sim.getVelocity() * IntakeConstants.intake_deploy_gear_ratio))

    def periodic(self) -> None:
        if is_simulation():
            self.update_sim()

        self._intake_table.putNumber("Intake Position", self.intake_deploy.get_position().value_as_double)
        self._intake_table.putString("Intake State", self.state)
        self._intake_table.putNumber("Intake Voltage", self.intake_deploy.get_motor_voltage(True).value_as_double)

        if 0 < self.intake_leader.get_motor_voltage().value_as_double < 0:
            self._intake_table.putBoolean("Intake Rollers On?", True)
        else:
            self._intake_table.putBoolean("Intake Rollers On?", False)
