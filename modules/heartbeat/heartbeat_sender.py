"""
Heartbeat sending logic.
"""

from pymavlink import mavutil
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class HeartbeatSender:
    """
    HeartbeatSender class to send a heartbeat
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> "tuple[bool, HeartbeatSender | None]":
        """
        Falliable create (instantiation) method to create a HeartbeatSender object.
        """
        try:
            instance = cls(cls.__private_key, connection, local_logger)
            return True, instance
        except (OSError, mavutil.mavlink.MAVError) as e:
            local_logger.error(f"Failed to create Heartbeat sender object: {e}")
            return False, None

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is HeartbeatSender.__private_key, "Use create() method"

        self.connection = connection
        self.local_logger = local_logger

    def run(
        self,
        local_logger: logger.Logger,
    ) -> None:
        """
        Attempt to send a heartbeat message.
        """
        try:
            self.connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
            )
            local_logger.info("Heartbeat sent!")
        except (OSError, mavutil.mavlink.MAVError) as e:
            local_logger.error(f"Failed to send heartbeat: {e}")


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
