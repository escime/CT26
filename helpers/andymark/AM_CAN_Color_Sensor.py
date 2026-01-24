import wpilib
from dataclasses import dataclass
from helpers.andymark.AMCanDevice import AMCanDevice
from helpers.andymark.AMCanDevice_Constants import (
    COLORSENSOR_COLORDATA_API,
    COLORSENSOR_PROX_TIMESTAMP_API,
    SET_REPORT_PERIOD_API,
    RESET_REPORT_PERIOD_API
)


class AMCANColorSensor(AMCanDevice):
    """
    Color sensor device over CAN for 2026 RobotPy.
    Provides access to color channels, proximity, and timestamp.
    """

    @dataclass
    class AMColorSensorData:
        """Aggregated readings from the color sensor."""
        clearC: int = 0
        red: int = 0
        green: int = 0
        blue: int = 0
        proximity: int = 0
        millisStamp: int = 0

    def __init__(self, device_id: int):
        # 15 = AndyMark vendor ID, 13 = color sensor type
        super().__init__(device_id, 15, 13)
        self._internal_data = self.AMColorSensorData()

    def get_data(self, timeout_ms: int = 150) -> AMColorSensorData:
        """
        Fetch the most recent color/proximity readings.
        :param timeout_ms: Maximum time to wait for each CAN frame.
        :return: A populated AM_ColorSensorData object.
        """
        # In Python, if we want to return the same persistent object (as in the Java
        # getData() vs getData(int timeout) logic), we update self._internal_data.
        # Otherwise, for a fresh copy, we could initialize a new object.
        d = self._internal_data

        color_data = wpilib.CANData()
        prox_data = wpilib.CANData()

        # Handle Color Channels API
        if self.receive_can_message(COLORSENSOR_COLORDATA_API, color_data, timeout_ms):
            b = color_data.data
            # Little-endian 16-bit reconstruction
            d.clearC = (b[1] << 8) | b[0]
            d.red = (b[3] << 8) | b[2]
            d.green = (b[5] << 8) | b[4]
            d.blue = (b[7] << 8) | b[6]

        # Handle Proximity and Timestamp API
        if self.receive_can_message(COLORSENSOR_PROX_TIMESTAMP_API, prox_data, timeout_ms):
            p = prox_data.data
            d.proximity = p[0]
            # Little-endian 32-bit timestamp reconstruction
            d.millisStamp = (p[4] << 24) | (p[3] << 16) | (p[2] << 8) | p[1]

        return d

    def set_report_period(self, period_ms: int) -> bool:
        """
        Request a new periodic report interval from the device.
        :param period_ms: Period in milliseconds (0-65535).
        """
        p = max(0, min(0xFFFF, period_ms))
        # Pack as 2-byte little-endian array
        data_bytes = bytes([
            p & 0xFF,
            (p >> 8) & 0xFF
        ])
        self.send_can_message(SET_REPORT_PERIOD_API, data_bytes)
        return True

    def reset_report_period(self) -> bool:
        """Reset the report period to the device default."""
        self.send_can_message(RESET_REPORT_PERIOD_API, b"")
        return True
