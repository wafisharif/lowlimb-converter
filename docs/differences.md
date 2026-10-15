# Changes from MyoConverter

osim2mujoco is based on MyoConverter at `cadf380`, which lives in `src/myoconverter/` with every modified file marked.
Each change below fixes a problem a check found, and each has a test that fails without it. Numbers follow the
station that found the problem; the findings files have the details.

## Running on current software

MyoConverter is pinned to MuJoCo 2.3.7 and NumPy 1. Four fixes get it running on MuJoCo 3, NumPy 2 and OpenSim 4.6
without Docker: `<option collision="predefined"/>` (removed in MuJoCo 3) is replaced, the now-sparse
`actuator_moment` is rebuilt as a dense matrix, `eq_active` is renamed and a missing dependency added, and a float
compared with `[]` (an error in NumPy 2) is fixed. {repo}`station1/FINDINGS.md`

## Model-level

- **1c.1** Gravity is taken from the model. MyoConverter used MuJoCo's default.
- **1c.2** CoordinateActuators keep their `optimal_force`. Rajagopal 2016's lumbar and arm actuators were 10× too weak
  without it.
- **1c.3** `<defaults>` is applied.

## Skeleton

- **2.1** Joints with an offset child frame rotate about the right point. Rajagopal 2016's knee was 15.6 mm off.
- **2.2** Numbers are written at full precision instead of 4 significant figures.
- **2.3** Spline couplings are fitted over the range the coordinate actually covers.
- **2.4** Where no single quartic follows the spline, two are chained through a helper joint. Rajagopal 2016's patella
  goes from 0.64° to 0.036°.
- **2.5** Polynomial axes up to degree 4 convert exactly, which makes RajagopalLaiUhlrich2023 convertible.

{repo}`station2/FINDINGS.md`

## Muscle paths

- **3.1** Conditional path points are fitted to OpenSim's path. MyoConverter left them always on, giving rectus
  femoris the wrong moment-arm sign at the knee.
- **3.2** Moving path points are fitted over the coordinate's range.
- **3.3** Moving-point axes that move less than 1 mm are fixed.
- **3.4** Wraps OpenSim allows on either side go round the shorter side. Rajagopal 2016's vasti were 127 mm too long
  in flexion.
- **3.5** Constrained wraps keep their side only where that matches OpenSim better. Iliacus and psoas were going the
  long way round the pelvis, up to 180 mm.

MyoConverter's own path optimisation only moves wrap sites, so it did nothing for either gait model.
{repo}`station3/FINDINGS.md`

## Muscle forces

- **4.1** After MyoConverter's particle swarm, each muscle is refitted against OpenSim's force at four activation
  levels. MyoConverter fitted full activation only and averaged 10–16% error, up to 121%.
- **4.2** Poses with no OpenSim equilibrium are excluded from the fit.
- **4.3** Results no longer depend on the swarm's random start, so left and right muscles match. MyoConverter's two
  copies of one muscle came out at 1.5% and 7.9% error.
- **4.4** Full activation and passive are each kept within 0.25% of their own best fit; the middle of the range takes
  the error a rigid tendon can't avoid.

{repo}`station4/FINDINGS.md`

## Walking

- **5.1** OpenSim's on/off jumps no longer drag conditional points off the path elsewhere.
- **5.2** Models are saved with a 2 ms time step. MyoConverter left 5 ms, which broke forward simulation.
- **5.3** A conditional point next to a fixed one can switch toward it through a helper joint.
- **5.4** The force fit skips the few poses where the path itself is wrong.

{repo}`station5/FINDINGS.md`

## MyoConverter's limitations

| | |
| --- | --- |
| 6.1 | Reads every OpenSim 4 file: `::` in element names, missing coordinate properties, any mesh-extension case, placeholder meshes, joints attached directly to bodies |
| 6.2 | Planar, Free, Ball and Gimbal joints |
| 6.3 | Exact masses and inertias (the template inflated small bodies) |
| 6.4 | CoordinateLimitForce |
| 6.5 | PathActuator and PathSpring |
| 6.6 | Contact spheres, half-spaces and Hunt–Crossley forces, with fitted parameters |
| 6.7 | ASCII STL and OBJ meshes |
| 6.8 | Half-spaces on moving bodies become boxes |
| 6.9 | Contact fitting handles coupled coordinates and locked platforms |
| 6.10 | No damping on coupling-driven joints (Rajagopal's kneecap lagged the knee by 1.9°) |
| 6.11 | Mesh files are found regardless of name case |
| 6.12 | Contact parameters must keep the whole model stable (the best fit launched the walker model 4 m up) |
| 6.13 | The contact fitting's OpenSim simulations run in a separate process (they occasionally failed in the main one) |
| 6.14 | A spline fit that hits a numerical breakdown keeps its best result instead of failing |

{repo}`station6/FINDINGS.md`

## Beyond the converter

- `osim2mujoco check` predicts whether a model will convert before trying.
- Every conversion can produce a validation report.
- `convert` refuses non-empty output folders. MyoConverter would quietly resume from whatever old results it found.
