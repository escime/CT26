from commands2 import Subsystem

from phoenix6.hardware import TalonFX, CANrange
from phoenix6.controls import VoltageOut
from phoenix6.configs import TalonFXConfiguration
from phoenix6.status_code import StatusCode
from phoenix6.utils import get_current_time_seconds, is_simulation
from phoenix6.canbus import CANBus

from ntcore import NetworkTableInstance
from wpilib import Servo

from constants import ClimberConstants


class ClimberSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self._debug_mode = False
        self._last_sim_time = get_current_time_seconds()
        self.state_values = ClimberConstants.state_values
        self.state = "stow"

        self._inst = NetworkTableInstance.getDefault()
        self._climber_table = self._inst.getTable("Climber")

        # CANRange Setup -----------------------------------------------------------------------------------------------
        # self._range_front = CANrange(ClimberConstants.ranger_front_can_id, CANBus("rio"))
        # self._range_back = CANrange(ClimberConstants.ranger_back_can_id, CANBus("rio"))

        # Servo Ratchet Setup ------------------------------------------------------------------------------------------
        self._servo = Servo(ClimberConstants.servo_port)

        # Climber Setup -------------------------------------------------------------------------------------------------
        self.climber = TalonFX(ClimberConstants.climber_can_id, CANBus("rio"))

        self.climber_volts = VoltageOut(0, True)

        climber_configs = TalonFXConfiguration()

        climber_configs.current_limits.stator_current_limit = ClimberConstants.stator_current_limit
        climber_configs.current_limits.stator_current_limit_enable = True
        climber_configs.current_limits.supply_current_limit = ClimberConstants.supply_current_limit
        climber_configs.current_limits.supply_current_limit_enable = True
        climber_configs.feedback.sensor_to_mechanism_ratio = ClimberConstants.gear_ratio
        climber_configs.motor_output.inverted = ClimberConstants.direction
        climber_configs.motor_output.neutral_mode = climber_configs.motor_output.neutral_mode.BRAKE

        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.climber.configurator.apply(climber_configs)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        self.climber.set_position(0)

        # Other Setup --------------------------------------------------------------------------------------------------
        self.last_time = get_current_time_seconds()

    def set_state(self, state: str) -> None:
        self.state = state
        self._servo.set(self.state_values[state][1])
        self.climber.set_control(self.climber_volts
                                 .with_output(self.state_values[state][0])
                                 .with_limit_forward_motion(self.get_upper_limit())
                                 .with_limit_reverse_motion(self.get_lower_limit()))

    def manual_control(self, axis: float) -> None:
        self.state = "manual"
        self.climber.set_control(self.climber_volts.with_output(12 * axis))

    def get_state(self) -> str:
        return self.state

    def get_upper_limit(self) -> bool:
        if self.climber.get_position().value_as_double >= ClimberConstants.deployed_position:
            return True
        else:
            return False

    def get_lower_limit(self) -> bool:
        if self.state == "climb":
            if self.climber.get_position().value_as_double <= ClimberConstants.climbed_position:
                return True
            else:
                return False
        else:
            if self.climber.get_position().value_as_double <= ClimberConstants.stowed_position:
                return True
            else:
                return False

    def get_range_back(self) -> float:
        # return self._range_back.get_distance().value_as_double
        return 0

    def get_range_front(self) -> float:
        # return self._range_front.get_distance().value_as_double
        return 0

    def set_debug_mode(self, on: bool) -> None:
        self._debug_mode = on

    def periodic(self) -> None:
        if self._debug_mode:
            self._climber_table.putNumber("Climber Position", self.climber.get_position().value_as_double)
            self._climber_table.putString("Climber State", self.get_state())
