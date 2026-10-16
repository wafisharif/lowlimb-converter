# Quick start

This uses `gait10dof18musc` from MyoConverter's repository, which is a leg model with 10 degrees of freedom and 18 muscles.

```bash
git clone https://github.com/MyoHub/myoconverter.git upstream
MODEL=upstream/models/osim/Gait10dof18musc/gait10dof18musc.osim
```

## Check the model

```bash
o2m check $MODEL --md check.md
```

Lists every element in the model and what will happen to it: converted, approximated, skipped, ignored, or unsupported.
Exit code 0 means it will convert, 1 means it will not (the report names the element it would fail on), and 2 means the file
could not be read.

## Convert

```bash
o2m convert $MODEL out/ --validate
```

Conversion runs in three steps:

1. XML conversion (seconds): bodies, joints, muscles, forces, and contacts.
2. Muscle paths (minutes): moving and conditional path points and wraps are fitted to OpenSim's muscle paths.
3. Muscle forces (minutes): muscle parameters are fitted to OpenSim's static forces at four activation levels.

The result is at `out/gait10dof18musc_cvt3.xml`. Use `--steps 1` to stop after step 1. The output folder has to be new
or empty.

Meshes are read from the `Geometry` folder next to the model. If they are somewhere else, pass `--geometry DIR`. For
opensim-models, where meshes are split between the top-level `Geometry` folder and each model's folder, copy both into
one folder and pass that.

## Read the report

`--validate` writes `out/validation/report.md` and compares the two models in about 1,000–2,000 poses in terms of body positions,
muscle lengths and moment arms, static muscle forces, and anything else the model has (mass properties, limit forces,
contacts). {doc}`report` explains the numbers.

## Add a walking trial

With an OpenSim inverse-kinematics result and the matching ground reaction forces, the report also replays the trial
in both simulators:

```bash
o2m validate $MODEL out/gait10dof18musc_cvt3.xml report/ \
    --ik subject01_walk_ik.mot --grf subject01_walk_grf.xml
```

OpenSim's example trials are under `Pipelines/` in opensim-models.

## Use the model

```python
import mujoco

model = mujoco.MjModel.from_xml_path("out/gait10dof18musc_cvt3.xml")
data = mujoco.MjData(model)
mujoco.mj_step(model, data)
```

Bodies, joints and muscles keep their OpenSim names. The default pose is the `default-pose` keyframe, and the meshes
are in `out/Geometry/`.
