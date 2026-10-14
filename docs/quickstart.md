# Quick start

This walks through converting one model and reading the result. It uses `gait10dof18musc` from MyoConverter's
repository, a 10-degree-of-freedom leg model with 18 muscles.

```bash
git clone https://github.com/MyoHub/myoconverter.git upstream
MODEL=upstream/models/osim/Gait10dof18musc/gait10dof18musc.osim
```

## 1. Check the model first

```bash
osim2mujoco check $MODEL --md check.md
```

This reads the model without converting it and lists every element (body, joint, muscle, path point, force, mesh)
with one status: **converted**, **approximated**, **skipped**, **ignored** or **unsupported**, plus the reason and
where in the converter it's handled. It takes seconds. The exit code says what will happen: 0 means the model will
convert, 1 means it won't (the report names the first element it would fail on), 2 means the file couldn't be read.

## 2. Convert it

```bash
osim2mujoco convert $MODEL out/ --validate
```

Conversion has three steps:

1. **XML conversion** (seconds): bodies, joints, muscles, forces and contacts become their MuJoCo equivalents.
2. **Muscle paths** (minutes): moving and conditional path points and wrapping are fitted to OpenSim's own muscle
   paths.
3. **Muscle forces** (minutes): each muscle's MuJoCo parameters are fitted to OpenSim's static force at four
   activation levels.

The result is `out/gait10dof18musc_cvt3.xml`. `--steps 1` stops after step 1 (`_cvt1.xml`; muscles not yet fitted).
The output folder must be new or empty: MyoConverter's pipeline would otherwise reuse old results it finds there
without saying so.

The geometry folder defaults to the `Geometry` folder next to the model. Pass `--geometry DIR` when the meshes are
elsewhere, for example for opensim-models, where they're split between the top-level `Geometry` folder and the
model's own: copy both into one folder and pass that.

## 3. Read the validation report

`--validate` writes `out/validation/report.md`. It compares the MuJoCo model with OpenSim in about 1000–2000 poses:

- **kinematics**: where every body is, in every pose (pass: within 1 mm and 0.5°);
- **paths**: every muscle's length, and its moment arm about every joint it crosses;
- **forces**: every muscle's static force at activation 0, 0.25, 0.5 and 1;
- **elements**: mass properties, limit forces, path actuators and springs, and contacts, where the model has them.

{doc}`report` explains each number and what counts as good.

## 4. Add a walking trial (optional)

If you have a walking trial for the model (OpenSim inverse-kinematics output and the ground reaction forces as an
OpenSim ExternalLoads file), the report can also replay it in both simulators:

```bash
osim2mujoco validate $MODEL out/gait10dof18musc_cvt3.xml report/ \
    --ik subject01_walk_ik.mot --grf subject01_walk_grf.xml
```

This adds muscle lengths and moment arms along the trial, and the joint moments needed to walk with the measured
forces. OpenSim's example trials are in opensim-models under `Pipelines/`.

## 5. Use the model

The converted model is a plain MuJoCo XML file:

```python
import mujoco

model = mujoco.MjModel.from_xml_path("out/gait10dof18musc_cvt3.xml")
data = mujoco.MjData(model)
mujoco.mj_step(model, data)
```

It keeps OpenSim's names for bodies, joints and muscles, its default pose as the `default-pose` keyframe, and the
converted meshes in `out/Geometry/`.
