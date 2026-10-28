#!/usr/bin/env python3
"""Run the simulator with owned process groups, readiness checks and cleanup."""

import argparse
import fcntl
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


class ProcessGroup:
    def __init__(self, directory):
        self.directory = directory
        self.children = []

    def start(self, name, command, *, cwd, env):
        with (self.directory / f'{name}.log').open('w') as output:
            child = subprocess.Popen(command, cwd=cwd, env=env, stdin=subprocess.DEVNULL,
                                     stdout=output, stderr=subprocess.STDOUT,
                                     start_new_session=True)
        self.children.append((name, child))
        return child

    def check(self):
        for name, child in self.children:
            if child.poll() is not None:
                raise RuntimeError(f'{name} exited ({child.returncode}); see {self.directory / (name + ".log")}')

    @staticmethod
    def session_groups(session_id):
        groups = set()
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
                if int(fields[3]) == session_id and fields[0] != 'Z':
                    groups.add(int(fields[2]))
            except (OSError, ValueError, IndexError):
                pass
        return groups

    def close(self):
        # Gazebo descendants can outlive a failed PX4 parent. Signal the whole
        # owned group even after its parent exits, without killing other sessions.
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            for _, child in reversed(self.children):
                # gdb puts its inferior in a separate process group within the
                # same session. Include those groups so debug runs also stop.
                for group_id in self.session_groups(child.pid):
                    try:
                        os.killpg(group_id, sig)
                    except ProcessLookupError:
                        pass
            deadline = time.monotonic() + (3.0 if sig != signal.SIGKILL else 1.0)
            while time.monotonic() < deadline:
                alive = False
                for _, child in self.children:
                    child.poll()
                    alive |= bool(self.session_groups(child.pid))
                if not alive:
                    return
                time.sleep(0.05)
        for _, child in self.children:
            child.wait(timeout=2.0)


def wait_for_telemetry(group, timeout):
    import math
    import rclpy
    from px4_msgs.msg import VehicleOdometry, VehicleStatus
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node('vyoma_simulation_readiness')
    samples = {'odometry': 0, 'status': 0}

    def odometry(message):
        if all(math.isfinite(value) for value in (*message.position, *message.velocity, *message.q)):
            samples['odometry'] += 1

    def status(message):
        samples['status'] += 1

    node.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', odometry,
                             qos_profile_sensor_data)
    node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v4', status,
                             qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + timeout
        while samples['odometry'] < 5 or samples['status'] < 2:
            group.check()
            if time.monotonic() >= deadline:
                raise RuntimeError(f'PX4 telemetry readiness timed out: {samples}')
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--px4-source', type=Path)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--training', action='store_true')
    parser.add_argument('--debug-px4', action='store_true', help='Capture a native backtrace with gdb')
    parser.add_argument('--duration', type=float, help='Stop this many seconds after telemetry readiness')
    parser.add_argument('--startup-timeout', type=float, default=90.0)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    px4_source = (args.px4_source or workspace / 'simulation/PX4-Autopilot').resolve()
    build = px4_source / 'build/px4_sitl_default'
    if not (build / 'bin/px4').is_file():
        parser.error(f'PX4 SITL is not built: {build / "bin/px4"}')
    if args.startup_timeout <= 0 or (args.duration is not None and args.duration <= 0):
        parser.error('timeouts must be positive')
    parent = workspace / 'artifacts/simulation'
    parent.mkdir(parents=True, exist_ok=True)
    lock = (parent / 'session.lock').open('w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('a simulation session already owns this workspace')
    directory = parent / f'{time.strftime("%Y%m%d-%H%M%S")}-{os.getpid()}'
    directory.mkdir()
    rootfs = directory / 'px4'
    rootfs.mkdir()
    (rootfs / 'etc').symlink_to(build / 'etc', target_is_directory=True)
    env = os.environ.copy()
    env.update(PX4_SIM_MODEL='gz_x500_depth', PX4_GZ_WORLD='training_world',
               PX4_GZ_MODELS=str(px4_source / 'Tools/simulation/gz/models'),
               PX4_GZ_WORLDS=str(px4_source / 'Tools/simulation/gz/worlds'),
               PX4_GZ_NO_FOLLOW='1', FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
    env.pop('PX4_GZ_STANDALONE', None)
    env.pop('PX4_GZ_MODEL_NAME', None)
    env['GZ_SIM_RESOURCE_PATH'] = ':'.join(filter(None, (
        env.get('GZ_SIM_RESOURCE_PATH'), env['PX4_GZ_MODELS'], env['PX4_GZ_WORLDS'])))
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = ':'.join(filter(None, (
        env.get('GZ_SIM_SYSTEM_PLUGIN_PATH'), str(build / 'src/modules/simulation/gz_plugins'))))
    env['GZ_SIM_SERVER_CONFIG_PATH'] = str(px4_source / 'src/modules/simulation/gz_bridge/server.config')
    if args.headless:
        env['HEADLESS'] = '1'
    else:
        env.pop('HEADLESS', None)
    shutil.copyfile(workspace / 'simulation/worlds/training_world.world',
                    Path(env['PX4_GZ_WORLDS']) / 'training_world.sdf')
    group = ProcessGroup(directory)

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    print(f'Simulation logs: {directory}', flush=True)
    try:
        group.start('dds', ['MicroXRCEAgent', 'udp4', '-p', '8888'], cwd=workspace, env=env)
        group.start('gcs', [sys.executable, str(workspace / 'scripts/simulation_gcs.py')],
                    cwd=workspace, env=env)
        group.start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                    '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/world/training_world/control@ros_gz_interfaces/srv/ControlWorld'],
                    cwd=workspace, env=env)
        px4_command = [str(build / 'bin/px4'), '-d']
        if args.debug_px4:
            px4_command = ['gdb', '--batch', '-ex', 'handle SIGCONT nostop noprint pass',
                           '-ex', 'handle SIGPIPE nostop noprint pass',
                           '-ex', 'run', '-ex', 'thread apply all bt',
                           '--args', *px4_command]
        group.start('px4', px4_command, cwd=rootfs, env=env)
        wait_for_telemetry(group, args.startup_timeout)
        print('PX4 odometry and vehicle status are streaming.', flush=True)
        group.start('autonomy', ['ros2', 'launch',
                    str(workspace / 'src/semantic_nav_pkg/launch/autonomous.launch.py'),
                    f'use_rviz:={str(not args.headless).lower()}',
                    f'training:={str(args.training).lower()}'], cwd=workspace, env=env)
        deadline = None if args.duration is None else time.monotonic() + args.duration
        while deadline is None or time.monotonic() < deadline:
            group.check()
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        group.close()
        lock.close()
        print('Simulation session stopped.', flush=True)


if __name__ == '__main__':
    main()
