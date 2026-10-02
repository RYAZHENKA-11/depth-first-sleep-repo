#!/usr/bin/env python3
"""Fixed controller of the stationary manipulator — thin wrapper over the shared driver.
Participants do NOT edit this; they command the arm remotely via go2_api (robot.arm)."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "common", "locomotion"))
from arm_driver_core import run   # noqa: E402

run()
