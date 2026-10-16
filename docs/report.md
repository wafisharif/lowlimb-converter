# The validation report

```bash
o2m validate model.osim out/model_cvt3.xml report/
```

Writes `report.md`, `report.json` (same content), one detailed JSON per level, and the OpenSim reference data
(`*_reference.npz`) so that the MuJoCo side can be rerun without OpenSim. Default levels: kinematics, paths, forces,
elements. Paths and forces need the `_cvt3.xml` model; `_cvt1.xml` does not have fitted muscles yet.

Both models are put in 1,000 random poses and a sweep of each coordinate, around 1,000–2,000 in
total. Coupled coordinates (a kneecap following the knee) follow their coupling in both. Each level first checks that
the two sides line up (same names + poses held) and reports the comparison as invalid if they do not.

## Kinematics

Position and orientation of every body in every pose. Pass: within 1 mm and 0.5° everywhere. This is the only level
with a fixed limit, since a properly converted skeleton should match almost exactly. The gait models come in at
0.22 mm, Rajagopal 2016 at 0.15 mm and 0.04°.

## Paths

Muscle lengths, and moment arms about every coordinate each muscle crosses. Pairs with a peak moment arm under 1 mm
are not scored. The report gives mean and median RMS error, the worst pair, and how many pairs are within 2 mm.

There is no standard pass mark. The tested models have mean moment-arm errors of 0.4–0.9 mm and worst pairs of
3.5–6.3 mm. Rajagopal 2016's gastrocnemius reaches 13 mm because its path wraps over two objects in a row (see
{doc}`limitations`).

## Forces

Each muscle is held still at activation 0, 0.25, 0.5 and 1 and its force compared with OpenSim's static equilibrium
force. Errors are RMS over poses, as a percentage of max isometric force. Poses where OpenSim has no equilibrium are
dropped and counted.

MuJoCo tendons are rigid and OpenSim's are elastic, so these can't match exactly. The fit favors full activation and
passive. Tested models: 1–3% passive, 2.5–3.6% at full activation.

## Elements

Whatever else the model has:

| Check | Pass |
| --- | --- |
| Mass, center of mass and inertia of every body | within 1e-9 |
| CoordinateLimitForce over a sweep of its coordinate | within 1% of peak |
| PathActuator and PathSpring length and force | 1 mm RMS, 2% |
| Signed distance between contact shapes | within 1e-6 m |
| Drops the contact calibration didn't use | reported only |

## Walking

Needs `--ik` (inverse-kinematics output) and `--grf` (ground reaction forces as an ExternalLoads file). Both models
follow the trial, and the report compares muscle lengths, moment arms, and the joint moments needed to walk with the
measured forces. Pass: joint moments within 0.02 N·m/kg at the hip, knee, ankle and lumbar joints.

`--start` and `--end` restrict it to part of the trial. `--controls` also replays recorded muscle excitations open
loop. That's reported only, since open-loop replays drift apart within milliseconds between any two simulators.

## Not checked

- Force–velocity behavior
- Forward simulation of whole movements, apart from the open-loop replay
- Anything `o2m check` lists as skipped or ignored
