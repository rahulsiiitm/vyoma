#!/usr/bin/env python3
"""Run independent, repeatable SITL missions for one policy or the baseline."""

import argparse
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import sys
import time


# Held-out points in the easy-goal region. This is not an obstacle benchmark.
EASY_GOALS = [(1.25, 0.0), (1.3, 0.2), (1.3, -0.2), (1.45, 0.2), (1.45, -0.2)]


def summarize(results, planned, model_path, provenance=None):
    count = len(results)
    return {'planned_cases': planned, 'completed_cases': count,
            'accepted_cases': sum(bool(item.get('success')) for item in results),
            'goal_and_landing_cases': sum(bool(item.get('lifecycle_completed')) for item in results),
            'model_path': str(model_path) if model_path else None,
            'model_sha256': hashlib.sha256(model_path.read_bytes()).hexdigest() if model_path else None,
            'provenance': provenance,
            'cases': results}


def capture_provenance(workspace, model_path):
    paths = ('src/semantic_nav_pkg/semantic_nav_pkg/mission_manager_node.py',
             'src/semantic_nav_pkg/semantic_nav_pkg/safety_supervisor_node.py',
             'src/semantic_nav_pkg/semantic_nav_pkg/depth_sector_node.py',
             'src/rl_navigation_pkg/rl_navigation_pkg/policy_node.py',
             'src/semantic_nav_pkg/config/autopilot_params.yaml',
             'simulation/worlds/training_world.world')
    hashes = {name: hashlib.sha256((workspace / name).read_bytes()).hexdigest() for name in paths}
    model_metadata = None
    if model_path:
        metadata_file = Path(str(model_path).removesuffix('.zip') + '.metadata.json')
        if metadata_file.is_file():
            model_metadata = json.loads(metadata_file.read_text(encoding='utf-8'))
    return {'configuration_sha256': hashes, 'model_metadata': model_metadata}


def stop_session(process):
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--px4-source', type=Path, required=True)
    parser.add_argument('--model-path', type=Path, help='Omit for the commissioning baseline')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--count', type=int, default=len(EASY_GOALS))
    parser.add_argument('--mission-timeout', type=float, default=240)
    parser.add_argument('--startup-timeout', type=float, default=120)
    parser.add_argument('--depth-rate', type=float, default=10)
    args = parser.parse_args()
    if not 1 <= args.count <= len(EASY_GOALS) or min(args.mission_timeout, args.startup_timeout, args.depth_rate) <= 0:
        parser.error('count must be 1–5 and timeouts and depth rate must be positive')
    workspace = args.workspace.resolve()
    model = args.model_path.resolve() if args.model_path else None
    if model is not None and not model.is_file():
        parser.error(f'model does not exist: {model}')
    provenance = capture_provenance(workspace, model)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    results = []
    summary_path = args.output_dir / 'summary.json'
    if summary_path.exists():
        parser.error(f'output already exists: {summary_path}')
    try:
        for index, (x, y) in enumerate(EASY_GOALS[:args.count], start=1):
            result_file = args.output_dir / f'case-{index}.json'
            # Never consume old output as the result of a new attempt.
            if result_file.exists():
                raise FileExistsError(f'output already exists: {result_file}')
            command = [sys.executable, str(workspace / 'scripts/simulation_session.py'),
                       '--workspace', str(workspace), '--px4-source', str(args.px4_source.resolve()),
                       '--headless', '--depth-rate', str(args.depth_rate),
                       '--startup-timeout', str(args.startup_timeout)]
            if model:
                command += ['--model-path', str(model)]
            session_log = args.output_dir / f'case-{index}-session.log'
            if session_log.exists():
                raise FileExistsError(f'output already exists: {session_log}')
            with session_log.open('w', encoding='utf-8') as log:
                process = subprocess.Popen(command, cwd=workspace, stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic() + args.startup_timeout + 15
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise RuntimeError(f'simulator exited; see {session_log}')
                    if 'PX4 odometry and vehicle status are streaming.' in session_log.read_text(encoding='utf-8'):
                        break
                    time.sleep(0.5)
                else:
                    raise RuntimeError(f'simulator startup timed out; see {session_log}')
                print(f'Case {index}: goal ({x}, {y})', flush=True)
                subprocess.run([sys.executable, str(workspace / 'scripts/mission_smoke.py'),
                                '--x', str(x), '--y', str(y), '--timeout', str(args.mission_timeout),
                                '--output', str(result_file)], cwd=workspace,
                               timeout=args.mission_timeout + 30, check=False)
                if not result_file.is_file():
                    raise RuntimeError(f'mission produced no result: {result_file}')
                results.append(json.loads(result_file.read_text(encoding='utf-8')))
            finally:
                stop_session(process)
            summary_path.write_text(json.dumps(summarize(results, args.count, model, provenance), indent=2) + '\n', encoding='utf-8')
    finally:
        summary_path.write_text(json.dumps(summarize(results, args.count, model, provenance), indent=2) + '\n', encoding='utf-8')
    print(f'Wrote {summary_path}', flush=True)


if __name__ == '__main__':
    main()
