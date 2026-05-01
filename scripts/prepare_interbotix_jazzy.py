#!/usr/bin/env python3
"""Prepare the Interbotix Jazzy source checkout for a ROS 2 colcon build."""

from pathlib import Path
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
JAZZY_REPOS = (
    REPO_ROOT / 'src' / 'interbotix_ros_core',
    REPO_ROOT / 'src' / 'interbotix_ros_manipulators',
    REPO_ROOT / 'src' / 'interbotix_ros_toolboxes',
)
OFFICIAL_IGNORE_MARKERS_TO_REMOVE = (
    REPO_ROOT
    / 'src'
    / 'interbotix_ros_core'
    / 'interbotix_ros_xseries'
    / 'COLCON_IGNORE',
)


def _git_branch(repo: Path) -> str:
    result = subprocess.run(
        ['git', '-C', str(repo), 'branch', '--show-current'],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return result.stdout.strip()


def main() -> int:
    missing_repos = [repo for repo in JAZZY_REPOS if not repo.exists()]
    if missing_repos:
        print('Missing Interbotix repositories:', file=sys.stderr)
        for repo in missing_repos:
            print(f'  {repo.relative_to(REPO_ROOT)}', file=sys.stderr)
        print(
            'Run: vcs import --recursive src < dependencies/interbotix_jazzy.repos',
            file=sys.stderr,
        )
        return 1

    for repo in JAZZY_REPOS:
        branch = _git_branch(repo)
        if branch != 'jazzy':
            print(
                f'Warning: {repo.relative_to(REPO_ROOT)} is on {branch!r}, '
                "expected 'jazzy'. Re-run vcs import --recursive.",
                file=sys.stderr,
            )

    for marker in OFFICIAL_IGNORE_MARKERS_TO_REMOVE:
        if marker.exists():
            marker.unlink()
            print(f'Removed {marker.relative_to(REPO_ROOT)}')
        else:
            print(f'{marker.relative_to(REPO_ROOT)} already removed')

    return 0


if __name__ == '__main__':
    sys.exit(main())
