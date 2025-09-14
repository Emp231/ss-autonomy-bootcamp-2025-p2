"""
Telemtry worker that gathers GPS data.
"""

import os
import pathlib
import time

from pymavlink import mavutil
from utilities.workers import queue_proxy_wrapper
from utilities.workers import worker_controller
from . import telemetry
from ..common.modules.logger import logger


# =================================================================================================
#                            ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
# =================================================================================================
def telemetry_worker(
    connection: mavutil.mavfile,
    queue: queue_proxy_wrapper.QueueProxyWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    queue is where the worker will communicate the status
    connection is the connection to the drone
    controller is how the communication happens
    """
    # =============================================================================================
    #                          ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
    # =============================================================================================

    # Instantiate logger
    worker_name = pathlib.Path(__file__).stem
    process_id = os.getpid()
    result, local_logger = logger.Logger.create(f"{worker_name}_{process_id}", True)
    if not result:
        print("ERROR: Worker failed to create logger")
        return

    # Get Pylance to stop complaining
    assert local_logger is not None

    local_logger.info("Logger initialized", True)

    # =============================================================================================
    #                          ↓ BOOTCAMPERS MODIFY BELOW THIS COMMENT ↓
    # =============================================================================================
    # Instantiate class object (telemetry.Telemetry)
    result, telemetry_obj = telemetry.Telemetry.create(
        connection=connection, local_logger=local_logger
    )
    if not result:
        local_logger.error("Failed to create telemetry object")
        return

    # Main loop: do work.
    while not controller.is_exit_requested():
        try:
            data = telemetry_obj.run()
            if data:
                local_logger.info(f"Telemetry data queued: {data}", True)
                queue.queue.put(data)
            else:
                local_logger.info("Telemetry run returned None", True)
        except (OSError, mavutil.mavlink.MAVError) as e:
            local_logger.error(f"Error when running main loop: {e}", True)

        time.sleep(0.01)

    local_logger.info("Worker has stopped")


# =================================================================================================
#                            ↑ BOOTCAMPERS MODIFY ABOVE THIS COMMENT ↑
# =================================================================================================
