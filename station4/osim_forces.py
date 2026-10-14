"""Wrapper so that station4/osim_forces.py keeps working. Its code moved on 2026-10-11 to
osim2mujoco/checks/osim_forces.py, so the installable tool can use it. Same command line; the module's names
are re-exported (tests import the package module directly, so that patching it takes effect).
Method and gates: station4/GATES.md.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from osim2mujoco.checks import osim_forces as _impl  # noqa: E402

globals().update({k: v for k, v in vars(_impl).items() if not (k.startswith("__") and k.endswith("__"))})

if __name__ == "__main__":
    sys.exit(_impl.main())
