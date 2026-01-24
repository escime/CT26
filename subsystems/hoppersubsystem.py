from commands2 import Subsystem

from phoenix6.hardware import TalonFX
from phoenix6.controls import VoltageOut, TorqueCurrentFOC
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
from constants import HopperConstants

class HopperSubsystem(Subsytem):
    def __init__(self):
        super().__init__()
        self._last_sim_time = get_current_time_seconds()
        self.state_values = HopperConstants.state_values
        self.state = "off"

        self._inst = NetworkTableInstance.getDefault()
        self._hopper_table = self._inst.getTable("Hopper")
        #Feeder motor
        #two indexer motors separate

        # Hopper Setup -----------------------------------------------------------------------------------------------
        self.left_indexer = TalonFX(HopperConstants.left_indexer_can_id, CANBus("rio"))
        self.right_indexer = TalonFX(HopperConstants.right_indexer_can_id, CANBus("rio"))

        self.hopper_volts = VoltageOut(0, True)

        indexer_configs = TalonFXConfiguration()

        indexer_configs.current_limits.stator_current_limit = HopperConstants.stator_current_limit
        indexer_configs.current_limits.stator_current_limit_enable = True
        indexer_configs.current_limits.supply_current_limit = HopperConstants.supply_current_limit
        indexer_configs.current_limits.supply_current_limit_enable = True
        indexer_configs.feedback.sensor_to_mechanism_ratio = HopperConstants.gear_ratio
        indexer_configs.motor_output.inverted = HopperConstants.direction

        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        status_2: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.left_indexer.configurator.apply(indexer_configs)
            status_2 = self.right_indexer.configurator.apply(indexer_configs)
            if status.is_ok() and status_2.is_ok():
                break
        if not status.is_ok() or not status_2.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        

        #Feeder Setup --------------------------------------------------------------------------------------------------
        self.feeder = TalonFX(HopperConstants.feeder_can_id, CANBus("rio"))
        self.feeder_volts = VoltageOut(0, True)

        # Other Setup --------------------------------------------------------------------------------------------------
        self.last_time = get_current_time_seconds()

    def set_state(self, state: str) -> None:
        self.state = state
        self.left_indexer.set_control(self.hopper_volts.with_output(self.state_values[state]))
        self.right_indexer.set_control(self.hopper_volts.with_output(self.state_values[state]))
        self.feeder.ser_control(self.feeder_volts.with_output(self.state_values[state]))

    def get_state(self) -> str:
        return self.state

    def update_sim(self):
        current_time = get_current_time_seconds()
        dt = current_time - self.last_time
        self.last_time = current_time

    def periodic(self) -> None:
        if is_simulation():
            self.update_sim()

        if 0 < self.intake_leader.get_motor_voltage().value_as_double < 0:
            self._hopper_table.putBoolean("Feeder Rollers On?", True)
        else:
            self._hopper_table.putBoolean("Feeder Rollers On?", False)


