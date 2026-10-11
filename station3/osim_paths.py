"""Wrapper so that station3/osim_paths.py keeps working. Its code moved on 2026-10-11 to
lowlimb_converter/checks/osim_paths.py, so the installable tool can use it. Same command line; the module's names
are re-exported (tests import the package module directly, so that patching it takes effect).
Method and gates: station3/GATES.md.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from lowlimb_converter.checks import osim_paths as _impl  # noqa: E402

globals().update({k: v for k, v in vars(_impl).items() if not (k.startswith("__") and k.endswith("__"))})

if __name__ == "__main__":
    sys.exit(_impl.main())
