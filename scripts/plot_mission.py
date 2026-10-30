#!/usr/bin/env python3
"""Plot a recorded PX4 odometry path over the configured Gazebo world."""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mission', type=Path)
    parser.add_argument('world', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    mission = json.loads(args.mission.read_text(encoding='utf-8'))
    points = mission.get('trajectory', [])
    if len(points) < 2:
        raise SystemExit('Mission has fewer than two recorded odometry samples')
    fig, ax = plt.subplots(figsize=(9, 7))
    for model in ET.parse(args.world).findall('./world/model'):
        pose = [float(x) for x in model.findtext('pose', '0 0 0 0 0 0').split()]
        geometry = model.find('./link/collision/geometry')
        if geometry is None:
            continue
        box, cylinder = geometry.find('box'), geometry.find('cylinder')
        if box is not None:
            sx, sy, _ = (float(x) for x in box.findtext('size').split())
            ax.add_patch(Rectangle((pose[0] - sx/2, pose[1] - sy/2), sx, sy,
                                   color='#a5b1c3' if model.get('name', '').startswith('wall_') else '#cc6b55',
                                   alpha=.7))
        elif cylinder is not None:
            ax.add_patch(Circle((pose[0], pose[1]), float(cylinder.findtext('radius')),
                                color='#cc6b55', alpha=.7))
    xs = [p['position'][0] for p in points]
    ys = [p['position'][1] for p in points]
    ax.plot(xs, ys, color='#2361a8', linewidth=2, label='Recorded odometry')
    ax.scatter(xs[0], ys[0], marker='o', color='#2361a8', s=65, label='Start', zorder=5)
    ax.scatter(xs[-1], ys[-1], marker='x', color='#c33737', s=90, label='Final position', zorder=5)
    ax.scatter(*mission['goal'], marker='*', color='#16a36a', s=150, label='Goal', zorder=5)
    ax.set(xlim=(-2.5, 10.5), ylim=(-5.5, 5.5), xlabel='East / map x (m)',
           ylabel='North / map y (m)', aspect='equal')
    ax.grid(alpha=.25)
    ax.legend(loc='upper right')
    outcome = 'completed' if mission.get('success') else mission.get('reason', 'failed')
    clearance = mission.get('minimum_navigation_clearance')
    detail = '' if clearance is None else f'; reported minimum clearance {clearance:.2f} m'
    ax.set_title(f'Mission path from simulated PX4 odometry: {outcome}{detail}')
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()
