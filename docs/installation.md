# Installation

You need Python 3.11–3.13 (OpenSim 4.6 has no wheels for other versions). Everything is tested on 3.12.

| Platform | Status |
| --- | --- |
| Linux x86-64 | Fully tested |
| macOS 15+ (Apple Silicon) | Installs and converts; the full checks haven't been run |
| Windows | Untested |

```bash
git clone https://github.com/wafisharif/osim2mujoco.git
cd osim2mujoco
python3.12 -m venv .venv
source .venv/bin/activate
pip install --require-hashes -r env/requirements.lock
pip install --no-deps .
o2m --version
```

The lockfile pins every dependency, including OpenSim 4.6 and MuJoCo 3.15, to the versions that the results were produced
with. Plain `pip install .` works too but may pull newer versions.

On a bare Linux machine, mesh conversion needs:

```bash
sudo apt-get install -y libgl1 libxrender1 libxext6
```

The command is `o2m`. `osim2mujoco` and `lowlimb-convert` (the project's name until October 2026) run the same
thing. The package isn't on PyPI yet, so install it from the repository as above.

## Models to try

- MyoConverter's repository has the two gait models used in testing: `models/osim/Gait10dof18musc/` and
  `models/osim/Gait2354Simbody/`.
- [opensim-models](https://github.com/opensim-org/opensim-models) has everything OpenSim distributes, including
  Rajagopal 2016 (`Models/Rajagopal/`) and example walking trials (`Pipelines/`).
