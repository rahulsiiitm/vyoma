#!/usr/bin/env python3
"""A heartbeat-only ground station for local SITL, never a flight controller."""

import time

from pymavlink import mavutil


def send_heartbeat(connection):
    connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)


def main():
    # Deliberately fixed to loopback and the pinned SITL instance's GCS port.
    # It sends no arming commands or parameter changes.
    connection = mavutil.mavlink_connection('udpout:127.0.0.1:18570',
                                           source_system=255, source_component=190)
    try:
        while True:
            send_heartbeat(connection)
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        connection.close()


if __name__ == '__main__':
    main()
