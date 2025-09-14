"""
Decision-making logic.
"""

import math

from pymavlink import mavutil
from ..common.modules.logger import logger
from ..telemetry import telemetry


class Position:
    """
    3D vector struct.
    """

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x
        self.y = y
        self.z = z


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
class Command:  # pylint: disable=too-many-instance-attributes
    """
    Command class to make a decision based on recieved telemetry,
    and send out commands based upon the data.
    """

    __private_key = object()

    @classmethod
    def create(
        cls,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> "tuple[True, Command] | tuple[False, None]":
        """
        Falliable create (instantiation) method to create a Command object.
        """
        try:
            command = cls(cls.__private_key, connection, target, local_logger)
            return True, command
        except (OSError, mavutil.mavlink.MAVError) as e:
            local_logger.error(f"Failed to create command object: {e}")
            return False, None

    def __init__(
        self,
        key: object,
        connection: mavutil.mavfile,
        target: Position,
        local_logger: logger.Logger,
    ) -> None:
        assert key is Command.__private_key, "Use create() method"

        self.connection = connection
        self.target = target
        self.local_logger = local_logger
        self.start_pos = None
        self.previous_data = None

    def run(
        self,
        data: telemetry.TelemetryData,
    ) -> None:
        """
        Make a decision based on received telemetry data.

        """

        messages = []

        velocity_vector = (data.x, data.y, data.z)
        self.local_logger.info(f"Velocity vector: {velocity_vector}", True)

        # alt
        da = self.target.z - data.z
        if abs(da) > 0.5:
            self.connection.mav.command_long_send(
                target_system=1,
                target_component=0,
                command=mavutil.mavlink.MAV_CMD_CONDITION_CHANGE_ALT,
                confirmation=0,
                param1=1,
                param2=0,
                param3=0,
                param4=0,
                param5=0,
                param6=0,
                param7=self.target.z,
            )
            msg = f"Change in Alt.: {da:.2f}"
            messages.append(msg)
            self.local_logger.info(msg, True)
            return [msg]

        # yaw
        dx = self.target.x - data.x
        dy = self.target.y - data.y
        desired_yaw = math.atan2(dy, dx)
        yaw_diff = desired_yaw - data.yaw
        yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi
        yaw_diff_deg = math.degrees(yaw_diff)

        if abs(yaw_diff_deg) > 5:
            self.connection.mav.command_long_send(
                target_system=1,
                target_component=0,
                command=mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                confirmation=0,
                param1=yaw_diff_deg,
                param2=5,
                param3=1,
                param4=1,
                param5=0,
                param6=0,
                param7=0,
            )
            msg = f"Change in Yaw: {yaw_diff_deg:.2f}"
            messages.append(msg)
            self.local_logger.info(msg, True)
            return [msg]

        return []


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
