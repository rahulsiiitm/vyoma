#!/usr/bin/env python3
"""Plot completed PPO episodes from a Stable-Baselines3 Monitor CSV."""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('monitor', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--algorithm', choices=('PPO', 'SAC'), default='PPO')
    args = parser.parse_args()
    with args.monitor.open(encoding='utf-8', newline='') as stream:
        next(stream)  # Monitor metadata line
        episodes = list(csv.DictReader(stream))
    if not episodes:
        raise SystemExit('No completed episodes in monitor file')
    rewards = [float(row['r']) for row in episodes]
    reached = [row.get('reached', '').lower() == 'true' for row in episodes]
    failed = [row.get('mission_failure', '').lower() == 'true' or
              row.get('collision', '').lower() == 'true' for row in episodes]
    fig, ax = plt.subplots(figsize=(10, 5.7))
    fig.subplots_adjust(left=0.10, right=0.97, top=0.88, bottom=0.22)
    fig.patch.set_facecolor('#f7f9fc')
    ax.set_facecolor('#f7f9fc')
    bars = ax.bar(range(1, len(episodes) + 1), rewards,
                  color=['#4a916b' if goal else '#d56a53' if failure else '#386ca7'
                         for goal, failure in zip(reached, failed)], width=0.62)
    ax.axhline(0, color='#68798a', linewidth=0.8)
    ax.grid(axis='y', alpha=0.2)
    ax.set_axisbelow(True)
    ax.set_title(f'{args.algorithm} pilot: completed simulation episodes', loc='left', fontsize=17, weight='bold')
    ax.set_xlabel('Episode')
    ax.set_ylabel('Episode reward')
    ax.set_xticks(range(1, len(episodes) + 1))
    margin = max(25.0, (max(rewards) - min(rewards)) * 0.15)
    for bar, row in zip(bars, episodes):
        value = float(row['r'])
        ax.text(bar.get_x() + bar.get_width()/2, value + (margin * 0.15 if value >= 0 else -margin * 0.15),
                f"{value:.0f}\n{row['l']} steps", ha='center',
                va='bottom' if value >= 0 else 'top', fontsize=9)
    ax.set_ylim(min(0, min(rewards)) - 2 * margin, max(0, max(rewards)) + 2 * margin)
    fig.text(0.1, 0.05, f'Green: goal   |   Orange: safety termination   |   Blue: other. '
             f'{sum(reached)} of {len(episodes)} training episodes reached the goal; not an evaluation.',
             fontsize=9, color='#41536a')
    for spine in ax.spines.values():
        spine.set_visible(False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160, facecolor=fig.get_facecolor())
    plt.close(fig)


if __name__ == '__main__':
    main()
