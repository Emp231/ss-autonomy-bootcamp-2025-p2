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
        self.time = 0
        self.x_velo = 0
        self.y_velo = 0
        self.z_velo = 0

    def run(
        self,
        data: telemetry.TelemetryData,
    ) -> None:
        """
        Make a decision based on received telemetry data.

        """

        self.time += 1
        self.x_velo += data.x_velocity
        self.y_velo += data.y_velocity
        self.z_velo += data.z_velocity
        avg_velo = (self.x_velo / self.time, self.y_velo / self.time, self.z_velo / self.time)

        self.local_logger.info(f"Average velocity: {avg_velo}")

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

            return f"ALT_CHANGE: {da}"

        # yaw
        dx = self.target.x - data.x
        dy = self.target.y - data.y
        desired_yaw = math.atan2(dy, dx)
        yaw_diff = desired_yaw - data.yaw
        yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi
        yaw_diff_deg = math.degrees(yaw_diff)

        if abs(yaw_diff_deg) > 5:
            if yaw_diff_deg > 0:
                direction = -1
            else:
                direction = 0

            self.connection.mav.command_long_send(
                target_system=1,
                target_component=0,
                command=mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                confirmation=0,
                param1=yaw_diff_deg,
                param2=5,
                param3=direction,
                param4=1,
                param5=0,
                param6=0,
                param7=0,
            )
            return f"YAW_CHANGE: {yaw_diff_deg}"

        return None


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
