from wpilib import CAN, Color8Bit, CANData

class AMCANColor:

    def __init__(self, can_id: int):
        """
        Software for accessing the AndyMark CAN Color & Proximity sensor.
        :param can_id: The assigned CAN ID of the color sensor by the AndyMark CAN Interface Utility.
        """
        self._device = CAN(can_id, 15, 13)
        self._can_buffer = CANData()


    def get_color(self) -> Color8Bit:
        self._device.readPacketLatest(20, self._can_buffer)
        return Color8Bit(red=self._can_buffer.data[3], green=self._can_buffer[5], blue=self._can_buffer[7])

    def get_proximity(self):
        self._device.readPacketLatest(1, self._can_buffer)
        return [self._can_buffer.data[0], self._can_buffer.data[1], self._can_buffer.data[2], self._can_buffer.data[3]]
