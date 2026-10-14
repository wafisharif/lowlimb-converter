# Installation

osim2mujoco needs Python 3.11–3.13, because OpenSim 4.6 publishes wheels only for those versions. It's tested on
Python 3.12, and the lockfile is for 3.12.

| Platform | Status |
| --- | --- |
| Linux (x86-64) | Tested end to end: every check in this documentation runs here, on every push |
| macOS 15 or newer (Apple Silicon) | Installs and converts (a quick test runs on every push); the full checks haven't been run there |
| Windows | OpenSim publishes wheels, but nothing has been tested |

## Install from the repository

```bash
git clone https://github.com/wafisharif/osim2mujoco.git
cd osim2mujoco
python3.12 -m venv .venv
source .venv/bin/activate
pip install --require-hashes -r env/requirements.lock   # the exact versions every check ran with
pip install --no-deps .                                   # adds the osim2mujoco command
osim2mujoco --version
```

The lockfile pins every package, including OpenSim 4.6 and MuJoCo 3.15, with hashes. Installing it first and the
package with `--no-deps` gives you the same environment the results in this documentation came from.
`pip install .` on its own also works, but resolves newer versions of the looser dependencies.

On a fresh Linux machine, mesh conversion (VTK) needs a few system libraries:

```bash
sudo apt-get install -y libgl1 libxrender1 libxext6
```

## The old name

Until October 2026 the project was called lowlimb-converter. Its command, `lowlimb-convert`, is still installed and
does the same as `osim2mujoco`.

## Models to try

- MyoConverter's repository ships the two gait models used throughout the checks:
  `models/osim/Gait10dof18musc/gait10dof18musc.osim` and `models/osim/Gait2354Simbody/gait2354.osim`, each with its
  `Geometry` folder.
- [opensim-models](https://github.com/opensim-org/opensim-models) has every model OpenSim distributes, including
  Rajagopal 2016 (`Models/Rajagopal/`) and the walking trials the walking check uses (`Pipelines/`). Its meshes are
  split between the repository's top-level `Geometry` folder and each model's own folder; see {doc}`quickstart` for
  how to point the converter at them.
