"""Command-line entry point for osim2mujoco/checks/mujoco_gait.py (Station 5;
method and gates: station5/GATES.md).

Same command line as `python -m osim2mujoco.checks.mujoco_gait`; works from a repo checkout without installing.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from osim2mujoco.checks import mujoco_gait as _impl  # noqa: E402

globals().update({k: v for k, v in vars(_impl).items() if not (k.startswith("__") and k.endswith("__"))})

if __name__ == "__main__":
    sys.exit(_impl.main())
