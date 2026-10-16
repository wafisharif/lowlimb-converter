# Commands

```text
osim2mujoco check    MODEL.osim [--geometry DIR] [--md FILE] [--json FILE]
osim2mujoco convert  MODEL.osim OUT_DIR [--geometry DIR] [--steps 1 2 3] [--validate] [...]
osim2mujoco validate MODEL.osim CONVERTED.xml OUT_DIR [--levels ...] [--ik FILE --grf FILE] [...]
```

`-v` / `--verbose`, placed before the command, prints the full pipeline and OpenSim log.

## check

Reports what will happen to each element of a model without converting it.

| Option | |
| --- | --- |
| `--geometry DIR` | Also check that every mesh file exists |
| `--md FILE` | Write the report as Markdown |
| `--json FILE` | Write the report as JSON |

| Status | Meaning |
| --- | --- |
| converted | Has an exact MuJoCo equivalent |
| approximated | Converted, but not exactly (e.g. a spline coupling becomes a polynomial) |
| skipped | Known but not converted; the converter warns (e.g. bushings) |
| ignored | Never read |
| unsupported | The conversion would fail here |

Exit codes: 0 will convert, 1 won't (the report names the first failing element), 2 couldn't read the file.

## convert

Runs `check` first and stops if the model won't convert.

| Option | |
| --- | --- |
| `OUT_DIR` | Output folder; must be new or empty |
| `--geometry DIR` | Mesh folder (default: `Geometry` next to the model) |
| `--steps 1 2 3` | 1 = XML conversion only; 1 2 3 = also fit muscle paths and forces (default) |
| `--validate` | Write a validation report into `OUT_DIR/validation` afterwards |
| `--speedy` | MyoConverter's faster, less thorough optimization |
| `--myoconverter-report` | Also produce MyoConverter's own plots and PDF (slow) |
| `--no-ground` | Don't add a ground plane (none is added if the model has its own contact anyway) |
| `--ik`, `--grf`, `--start`, `--end`, `--controls` | Walking trial for the report (see `validate`) |

Output for `model.osim`:

| File | |
| --- | --- |
| `model_cvt1.xml` | After step 1 |
| `model_cvt2.xml` | After step 2 |
| `model_cvt3.xml` | Final model |
| `Geometry/` | Meshes (binary STL or OBJ) |
| `contact_calibration.json` | Fitted contact parameters, for models with contact |
| `model_conversion.log` | Full log |
| `validation/` | Validation report, with `--validate` |

## validate

Compares a converted model with the original. See {doc}`report`.

| Option | |
| --- | --- |
| `XML` | Converted model (use `_cvt3.xml` for paths and forces) |
| `OUT_DIR` | Where `report.md`, `report.json` and the detailed results go |
| `--levels` | Any of `kinematics`, `paths`, `forces`, `elements`, `walking`; default is all but `walking` |
| `--ik FILE` | Inverse-kinematics result of a walking trial (`.mot` or `.sto`) |
| `--grf FILE` | Its ground reaction forces (OpenSim ExternalLoads `.xml`) |
| `--start`, `--end` | Part of the trial to compare, in seconds |
| `--controls FILE` | Muscle excitations (`.sto`) to replay open loop; reported only |

`--ik` and `--grf` together add the walking level.

## Running checks individually

Each check has an OpenSim side, which writes a reference `.npz`, and a MuJoCo side that compares against it without
needing OpenSim:

```bash
python -m osim2mujoco.support model.osim --geometry Geometry/ --md report.md
python -m osim2mujoco.checks.osim_reference model.osim reference.npz
python -m osim2mujoco.checks.mujoco_compare model_cvt3.xml reference.npz out.json
```

All modules are listed in {doc}`python-api`.
