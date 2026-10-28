import importlib.util
import io
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[1]


def test_training_world_has_estimator_environment_and_flight_level_obstacles():
    world = ET.parse(ROOT / 'simulation/worlds/training_world.world').getroot().find('world')
    assert any(float(value) != 0.0 for value in world.findtext('magnetic_field').split())
    assert world.find('spherical_coordinates/latitude_deg') is not None
    for name in ('box1', 'box2', 'cylinder_1', 'wall_n', 'wall_s', 'wall_e', 'wall_w'):
        model = world.find(f"model[@name='{name}']")
        center_z = float(model.findtext('pose').split()[2])
        geometry = model.find('link/collision/geometry')
        height = (float(geometry.findtext('box/size').split()[2])
                  if geometry.find('box') is not None else float(geometry.findtext('cylinder/length')))
        assert center_z - height / 2 <= 5.0 <= center_z + height / 2


@pytest.fixture
def process_group(tmp_path):
    if os.name != 'posix':
        pytest.skip('simulation process ownership uses Linux process groups')
    spec = importlib.util.spec_from_file_location('simulation_session', ROOT / 'scripts/simulation_session.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    group = module.ProcessGroup(tmp_path)
    yield group
    group.close()


def test_session_shutdown_stops_owned_process(process_group, tmp_path):
    child = process_group.start('worker', [sys.executable, '-c', 'import time; time.sleep(60)'],
                                cwd=tmp_path, env=os.environ.copy())
    process_group.check()
    process_group.close()
    assert child.poll() is not None


def test_session_detects_component_exit(process_group, tmp_path):
    child = process_group.start('failed', [sys.executable, '-c', 'raise SystemExit(7)'],
                                cwd=tmp_path, env=os.environ.copy())
    child.wait(timeout=5)
    with pytest.raises(RuntimeError, match=r'failed exited \(7\)'):
        process_group.check()


def test_simulation_ground_station_only_sends_gcs_heartbeat():
    mavutil = pytest.importorskip('pymavlink.mavutil')
    spec = importlib.util.spec_from_file_location('simulation_gcs', ROOT / 'scripts/simulation_gcs.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    output = io.BytesIO()
    encoder = mavutil.mavlink.MAVLink(output, srcSystem=255, srcComponent=190)
    module.send_heartbeat(SimpleNamespace(mav=encoder))
    decoder = mavutil.mavlink.MAVLink(None)
    message = decoder.parse_char(output.getvalue())
    assert message.get_type() == 'HEARTBEAT'
    assert message.type == mavutil.mavlink.MAV_TYPE_GCS
    assert message.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID
    assert message.base_mode == 0


def test_shutdown_includes_descendant_in_different_process_group(process_group, tmp_path):
    code = ('import os, subprocess, sys, time; '
            'p = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"], '
            'preexec_fn=os.setpgrp); print(p.pid, flush=True); time.sleep(60)')
    process_group.start('debugger', [sys.executable, '-c', code],
                         cwd=tmp_path, env=os.environ.copy())
    output = tmp_path / 'debugger.log'
    deadline = time.monotonic() + 5
    while not output.read_text().strip() and time.monotonic() < deadline:
        time.sleep(0.01)
    descendant = int(output.read_text().strip())
    process_group.close()
    stat = Path(f'/proc/{descendant}/stat')
    assert not stat.exists() or stat.read_text().rsplit(')', 1)[1].split()[0] == 'Z'
