#!/usr/bin/env python3
"""Measure one commissioned SITL mission; report failure instead of assuming success."""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, default=1.5)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--observe-only', action='store_true', help='Measure an already submitted mission')
    parser.add_argument('--landing-tolerance', type=float, default=0.75)
    parser.add_argument('--output', type=Path,
                        default=Path('artifacts') / f'mission-{time.strftime("%Y%m%d-%H%M%S")}.json')
    args = parser.parse_args()
    if (not all(math.isfinite(value) for value in (args.x, args.y, args.timeout, args.landing_tolerance))
            or args.timeout <= 0 or args.landing_tolerance <= 0):
        parser.error('goal and timeout must be finite; timeout must be positive')
    rclpy.init()
    node = rclpy.create_node('vyoma_mission_smoke', parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    result = {'acceptance_version': 2, 'landing_tolerance': args.landing_tolerance,
              'goal': [args.x, args.y], 'success': False, 'states': [],
              'armed_seen': False, 'maximum_altitude': 0.0, 'path_length': 0.0,
              'minimum_navigation_clearance': None}
    state = {'connected': False, 'status_time': 0.0, 'position': None,
             'requested': args.observe_only}
    publisher = node.create_publisher(PoseStamped, '/vyoma/mission/command', 10)

    def mission(message):
        value = json.loads(message.data)
        if state['requested'] and (not result['states'] or result['states'][-1]['state'] != value['state']):
            result['states'].append(value)
            print(f"Mission: {value['state']} ({value['reason']})", flush=True)

    def status(message):
        value = json.loads(message.data)
        state['connected'] = value.get('connected', False)
        state['status_time'] = time.monotonic()
        if state['requested']:
            result['armed_seen'] |= value.get('arming_state') == 2

    def odometry(message):
        point = message.pose.pose.position
        position = (point.x, point.y, point.z)
        if not all(math.isfinite(value) for value in position):
            return
        if state['requested']:
            result['maximum_altitude'] = max(result['maximum_altitude'], point.z)
            if state['position'] is not None:
                result['path_length'] += math.dist(position, state['position'])
            result['final_distance'] = math.hypot(args.x - point.x, args.y - point.y)
        state['position'] = position

    def safety(message):
        value = json.loads(message.data)
        if result['states'] and result['states'][-1]['state'] in (
                'RL_NAVIGATE', 'OBSTACLE_DEFLECTION', 'PATH_RECOVERY'):
            clearance = value.get('minimum_clearance')
            if clearance is not None and math.isfinite(clearance):
                old = result['minimum_navigation_clearance']
                result['minimum_navigation_clearance'] = clearance if old is None else min(old, clearance)

    node.create_subscription(String, '/vyoma/mission/state', mission, 10)
    node.create_subscription(String, '/vyoma/px4/status', status, 10)
    node.create_subscription(Odometry, '/vyoma/odometry', odometry, 10)
    node.create_subscription(String, '/vyoma/safety/status', safety, 10)
    started = time.monotonic()
    requested_at = started if args.observe_only else None
    try:
        while time.monotonic() - (requested_at or started) < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (not state['requested'] and state['connected'] and state['position'] is not None
                    and time.monotonic() - state['status_time'] < 1.0
                    and publisher.get_subscription_count() >= 2):
                goal = PoseStamped()
                goal.header.stamp = node.get_clock().now().to_msg()
                goal.header.frame_id = 'map'
                goal.pose.position.x, goal.pose.position.y = args.x, args.y
                goal.pose.orientation.w = 1.0
                state['requested'] = True
                requested_at = time.monotonic()
                publisher.publish(goal)
            phases = [value['state'] for value in result['states']]
            if phases and phases[-1] == 'DISARMED' and any(phase != 'DISARMED' for phase in phases):
                result['lifecycle_completed'] = (result['armed_seen'] and 'GOAL_HOLD' in phases
                                                 and 'LAND' in phases and 'FAILSAFE_LAND' not in phases)
                result['success'] = (result['lifecycle_completed']
                                     and result.get('final_distance', math.inf) <= args.landing_tolerance)
                result['reason'] = 'completed' if result['success'] else 'mission_aborted'
                if result['lifecycle_completed'] and not result['success']:
                    result['reason'] = 'landing_outside_goal_tolerance'
                break
        else:
            result['reason'] = 'timeout' if state['requested'] else 'runtime_not_ready'
    finally:
        result['wall_seconds'] = time.monotonic() - started
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n')
        print(f'Result: {args.output} (success={result["success"]})', flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
