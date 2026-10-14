# The validation report

`osim2mujoco validate` (or `convert --validate`) compares a converted model with its OpenSim original and writes:

- `report.md`: the summary, readable;
- `report.json`: the same summary, for scripts;
- one detailed JSON per level (`kinematics.json`, `paths.json`, ...), with the result for every body, muscle or pair;
- the OpenSim reference data (`*_reference.npz`), so the MuJoCo side can be rechecked without OpenSim.

```bash
osim2mujoco validate model.osim out/model_cvt3.xml report/ [--levels kinematics paths forces elements walking]
```

The default levels are kinematics, paths, forces and elements. Use the `_cvt3.xml` model for paths and forces: the
`_cvt1.xml` model's muscles aren't fitted yet.

## How the comparison works

Both models are put in the same poses: 1000 random poses within every coordinate's range, plus a sweep of every
coordinate on its own (about 1000–2000 poses in all). Coordinates that a coupling drives (a kneecap that follows the
knee, say) follow it in both models. Everything is compared pose by pose.

Each level first checks that the comparison itself is fair: the same names on both sides, poses held exactly. If that
fails, the level says the comparison isn't valid instead of reporting numbers.

## Levels

### Kinematics

Where every body is, and how it's turned, in every pose. **Pass: within 1 mm and 0.5° in every pose.** This is the one
level with an absolute pass/fail limit, because a correct conversion of the skeleton should be essentially exact.

The report gives the worst position and orientation error with the body and pose it happened in, and the RMS over all
bodies and poses. For reference, the gait models are within 0.22 mm and Rajagopal 2016 within 0.15 mm and 0.04°.

### Paths

Every muscle's length, and its moment arm about every coordinate it crosses (how much the muscle shortens as the joint
turns, which sets how much joint torque its force makes). Only muscle–coordinate pairs with a peak moment arm of at
least 1 mm are scored. The report gives the mean and median RMS error over pairs, the worst pair (in mm and as a
percentage of its peak), and how many pairs are within 2 mm.

There's no absolute pass/fail limit, because no published standard exists: the project judged paths against
MyoConverter's result for the same model, and the report shows the project's own results as a reference. As a rule
of thumb, the checked models have mean moment-arm errors of 0.4–0.9 mm and worst pairs of 3.5–6.3 mm (gait models),
or 13 mm for a pair that wraps over two objects in a row in OpenSim (Rajagopal 2016's gastrocnemius; see
{doc}`limitations`).

### Forces

Every muscle held still in every pose at activation 0 (passive), 0.25, 0.5 and 1, and its force compared with
OpenSim's static equilibrium force. Errors are RMS over poses, as a percentage of the muscle's maximum isometric
force. Poses where OpenSim has no static equilibrium are left out, and counted.

MuJoCo's muscles have rigid tendons and OpenSim's are elastic, so a perfect match isn't possible; the fit keeps full
activation and passive closest. The checked models have mean errors of 1–3% passive and 2.5–3.6% at full activation.

### Elements

Everything else the model has:

- **mass properties**: every body's mass, centre of mass and inertia (pass: within 1e-9, i.e. exact);
- **limit forces** (`CoordinateLimitForce`): the generalized force over a sweep of the coordinate (pass: within 1% of
  its peak);
- **path actuators and springs**: length and force (pass: length within 1 mm RMS, force within 2%);
- **contact geometry**: the signed distance between every pair of contact shapes (pass: within 1e-6 m);
- **contact drops**: for a model that can be dropped (a root joint with a free translation), the drops the converter
  didn't calibrate on. These are reported, not passed or failed: see {doc}`limitations`.

### Walking (with a trial)

Give `--ik` (OpenSim inverse-kinematics output, `.mot` or `.sto`) and `--grf` (the ground reaction forces as an OpenSim
ExternalLoads `.xml`). Both models follow the trial frame by frame, and the report gives:

- every muscle's length and moment arms along the trial;
- the joint moments needed to walk with the measured forces (inverse dynamics in both simulators). **Pass: within
  0.02 N·m per kg of body mass** at the hip, knee, ankle and lumbar joints.

`--start`/`--end` limit the comparison to part of the trial. `--controls` with recorded muscle excitations (e.g.
OpenSim CMC's `*_controls.sto`) also replays them open loop in both simulators. That's reported only: open-loop
replay drifts apart within milliseconds in any pair of simulators, MyoConverter's conversions included.

## What the report doesn't check

- How muscle force changes with speed (force–velocity).
- Forward simulation of whole movements, apart from the optional open-loop replay.
- Anything the support report lists as skipped or ignored (run `osim2mujoco check` to see those).
