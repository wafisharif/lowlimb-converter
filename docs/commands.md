# Commands

```text
osim2mujoco check    MODEL.osim [--geometry DIR] [--md FILE] [--json FILE]
osim2mujoco convert  MODEL.osim OUT_DIR [--geometry DIR] [--steps 1 2 3] [--validate] [...]
osim2mujoco validate MODEL.osim CONVERTED.xml OUT_DIR [--levels ...] [--ik FILE --grf FILE] [...]
```

`osim2mujoco --version` prints the version, and `-v` / `--verbose` (before the command) shows the full log of the
conversion pipeline and OpenSim.

## check

Lists every element of an OpenSim model and whether it converts. It doesn't convert anything and takes seconds.

| Option | What it does |
| --- | --- |
| `--geometry DIR` | Mesh folder: also checks that every mesh file exists (up to case, as the converter does) |
| `--md FILE` | Write the report as Markdown |
| `--json FILE` | Write the report as JSON |

Every element gets one status:

| Status | Meaning |
| --- | --- |
| converted | Becomes the MuJoCo equivalent |
| approximated | Converted, but not exactly (e.g. a spline joint coupling becomes a polynomial fit) |
| skipped | The converter knows the element but doesn't convert it, and says so (e.g. bushing forces) |
| ignored | Not read at all (e.g. display settings) |
| unsupported | The converter would fail on it |

Exit codes: **0** the model will convert, **1** it won't (the report names the first element it fails on), **2** the
file couldn't be read. The report also names the converter source that handles each element, so you can see why.

## convert

Converts a model. It runs `check` first and refuses a model that won't convert, with the reason, before doing
anything.

| Option | What it does |
| --- | --- |
| `OUT_DIR` | Output folder; must be new or empty |
| `--geometry DIR` | Mesh folder (default: the `Geometry` folder next to the model) |
| `--steps 1 2 3` | Which steps to run. 1 = XML conversion only (seconds); 1 2 3 = also fit muscle paths and forces (minutes; the default) |
| `--validate` | Then write a validation report into `OUT_DIR/validation` |
| `--speedy` | MyoConverter's faster, less thorough optimisation |
| `--myoconverter-report` | Also run MyoConverter's own validation plots and PDF (slow) |
| `--no-ground` | Don't add a ground plane (one is added unless the model has its own contact) |
| `--ik`, `--grf`, `--start`, `--end`, `--controls` | A walking trial for the validation report (see `validate`) |

Output, for a model `model.osim`:

| File | What |
| --- | --- |
| `model_cvt1.xml` | After step 1 (muscle paths and forces not fitted yet) |
| `model_cvt2.xml` | After step 2 (muscle paths fitted) |
| `model_cvt3.xml` | The converted model |
| `Geometry/` | Meshes, as binary STL (or OBJ) |
| `contact_calibration.json` | For a model with contacts: the fitted contact parameters and how well they match OpenSim's drops |
| `model_conversion.log` | The full log |
| `validation/` | With `--validate`: the validation report |

## validate

Compares a converted model with its OpenSim original; see {doc}`report` for what each number means.

| Option | What it does |
| --- | --- |
| `XML` | The converted model (the `_cvt3.xml` for paths and forces) |
| `OUT_DIR` | Folder for `report.md`, `report.json` and the detailed results |
| `--levels` | Any of `kinematics`, `paths`, `forces`, `elements`, `walking` (default: all but walking) |
| `--ik FILE` | OpenSim inverse-kinematics result of a walking trial (`.mot` or `.sto`) |
| `--grf FILE` | Its ground reaction forces, as an OpenSim ExternalLoads file (`.xml`) |
| `--start`, `--end` | The part of the trial to compare, in seconds (default: all of it) |
| `--controls FILE` | Muscle excitations (`.sto`, e.g. CMC's `*_controls.sto`): also replay them open loop (reported only) |

Giving `--ik` and `--grf` adds the walking level.

## Running a module directly

The support report and each check also run on their own, which helps when debugging a single model:

```bash
python -m osim2mujoco.support model.osim --geometry Geometry/ --md report.md
python -m osim2mujoco.checks.osim_reference model.osim reference.npz          # OpenSim side of kinematics
python -m osim2mujoco.checks.mujoco_compare model_cvt3.xml reference.npz out.json
```

Every check is split the same way: an OpenSim side writes a reference `.npz`, and a MuJoCo side compares against it
without needing OpenSim. The modules are listed in {doc}`python-api`.
