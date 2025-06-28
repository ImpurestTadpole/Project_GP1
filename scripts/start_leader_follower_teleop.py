#!/usr/bin/env python3
"""Launch a robot in *Follower* mode for leader–follower teleoperation.

This script is a small wrapper around *start_teleop.py* that preselects the
`leader_follower` control backend so the robot mirrors whatever the leader
publishes on the `/leader` topics.

Example
-------
$ python3 scripts/start_leader_follower_teleop.py --robot ur5
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
base_script = THIS_DIR / "start_teleop.py"

# Pass through user arguments but enforce --control leader_follower
cmd = [sys.executable, str(base_script), "--control", "leader_follower", *sys.argv[1:]]

subprocess.run(cmd) 