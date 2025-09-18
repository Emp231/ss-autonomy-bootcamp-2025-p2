"""
Heartbeat receiving logic.
"""

import time
from pymavlink import mavutil
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class HeartbeatReceiver:
    """
    HeartbeatReceiver class to send a heartbeat
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> "tuple[True, HeartbeatReceiver] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a HeartbeatReceiver object.
        """
        try:
            receiver = cls(cls.__private_key, connection, local_logger)
            return True, receiver
        except (OSError, mavutil.mavlink.MAVError) as e:
            local_logger.error(f"Failed to create Heartbeat receiver object: {e}")
            return False, None

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        local_logger: logger.Logger,
    ) -> None:
        assert key is HeartbeatReceiver.__private_key, "Use create() method"

        self.connection = connection
        self.local_logger = local_logger
        self.missed_heartbeats = 0
        self.status = "Disconnected"

    def run(self) -> None:
        """
        Attempt to recieve a heartbeat message.
        If disconnected for over a threshold number of periods,
        the connection is considered disconnected.
        """
        try:
            msg = self.connection.recv_match(type="HEARTBEAT", blocking=False)

            if msg is not None:
                if self.status != "Connected":
                    self.local_logger.info("Reconnected", True)
                self.missed_heartbeats = 0
                self.status = "Connected"
                self.local_logger.info(f"Status: {self.status}", True)
            else:
                self.missed_heartbeats += 1
                self.local_logger.warning(
                    f"Did not receive heartbeat. Count: {self.missed_heartbeats}", True
                )
                self.status = "Disconnected"

                if self.missed_heartbeats >= 5:
                    self.status = "Disconnected"
                    self.local_logger.warning("Lost connection", True)

        except (OSError, mavutil.mavlink.MAVError) as e:
            self.local_logger.error(f"Error while trying to receive message: {e}", True)

        time.sleep(1)


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
