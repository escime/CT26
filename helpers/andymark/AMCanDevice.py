import wpilib
from helpers.andymark.AMDiagnosticsServer import AMDiagnosticsServer

class AMCanDevice:
    """
    Base class for AndyMark CAN devices on the FRC roboRIO CAN bus.
    Wraps RobotPy's CAN class to provide transport and device identity.
    """

    def __init__(self, device_id: int, device_manufacturer: int, device_type: int):
        # Ensure the diagnostics server is running for this 2026 session
        AMDiagnosticsServer.ensure_server_running()

        self._can = wpilib.CAN(device_id, device_manufacturer, device_type)
        self._device_id = device_id
        self._manufacturer = device_manufacturer
        self._type = device_type

    def send_can_message(self, api_id: int, data: bytes = None):
        """
        Low-level helper to transmit a CAN packet.
        In Python, an empty payload is handled by an empty bytes object.
        """
        if data is None:
            data = b""
        self._can.writePacket(data, api_id)

    def receive_can_message(self, api_id: int, data: wpilib.CANData, timeout_ms: int) -> bool:
        """
        Receive a packet with a specific API ID, optionally waiting up to a timeout.
        Wraps wpilib.CAN.readPacketTimeout.
        """
        # Returns True if a matching frame was received within timeout
        return self._can.readPacketTimeout(api_id, timeout_ms, data)

    def restart_device(self):
        """
        Request a device reboot via vendor-specific CAN command.
        Uses the RESTART_DEVICE_API constant.
        """
        # Assuming AMCanDevice_Constants.RESTART_DEVICE_API is imported or defined
        from helpers.andymark.AMCanDevice_Constants import RESTART_DEVICE_API
        self.send_can_message(RESTART_DEVICE_API)

    @property
    def device_id(self) -> int:
        """Get this device's configured CAN ID."""
        return self._device_id
