# How it differs from MyoConverter

osim2mujoco starts from MyoConverter at commit `cadf380`, vendored in `src/myoconverter/` with every changed file
marked. Each change below came from a check that found a problem, and each has a test that fails on the converter
before it. The numbering follows the station that found it (see {doc}`validation`). The findings files have the
details and the numbers.

## Running on a current stack (Station 1a)

MyoConverter is pinned to MuJoCo 2.3.7 and NumPy 1. Four small fixes make it run on MuJoCo 3, NumPy 2 and OpenSim 4.6
without Docker:

- `<option collision="predefined"/>` was removed in MuJoCo 3, and converted models no longer loaded. It's replaced by
  MuJoCo's documented equivalent.
- `actuator_moment` became sparse in MuJoCo 3, a silent risk. The dense matrix is rebuilt.
- A renamed model field (`eq_active` → `eq_active0`), and a missing dependency (`networkx`).
- The particle-swarm optimiser compared a float with `[]`, which NumPy 2 rejects.

Compiled models match MyoConverter's own, apart from documented differences that don't change the physics.
Details: {repo}`station1/FINDINGS.md`.

## Model-level fixes (1c)

- **1c.1 Gravity.** The model's gravity is converted (MyoConverter used MuJoCo's default 9.81 m/s²).
- **1c.2 Torque actuators' strength.** CoordinateActuators keep their `optimal_force` (Rajagopal 2016's lumbar and arm
  actuators came out 10× too weak).
- **1c.3 `<defaults>`.** Properties a component omits come from the file's `<defaults>`, as in OpenSim (MyoConverter's
  documented limitation 3).

## Skeleton (Station 2)

- **2.1 Joint frames.** A joint with an offset child frame now rotates about the right point (Rajagopal 2016's knee
  was 15.6 mm off).
- **2.2 Precision.** Numbers are written in full, not with 4 significant figures.
- **2.3 Spline fits.** A spline coupling (a kneecap following the knee) is fitted over the range the coordinate
  actually moves through.
- **2.4 Chained couplings.** Where no single quartic can follow the spline (Rajagopal 2016's patella: 0.64° at best),
  two quartics run in sequence through a helper joint (0.036°).
- **2.5 Polynomial axes.** `PolynomialFunction` axes up to degree 4 convert exactly (RajagopalLaiUhlrich2023 couldn't
  be converted before).

Details: {repo}`station2/FINDINGS.md`.

## Muscle paths (Station 3)

- **3.1 Conditional path points** are fitted to OpenSim's own path. MyoConverter kept them always on, which gave
  rectus femoris the wrong moment-arm sign at the knee (69–78 mm error).
- **3.2 Moving path points** are fitted over the coordinate's range.
- **3.3** A moving-point axis that moves less than 1 mm is held fixed.
- **3.4 Wrap sides.** A wrap object that OpenSim lets the path pass on either side gets no side site, so MuJoCo also
  takes the shorter side (Rajagopal 2016's vasti were 127 mm too long in flexion).
- **3.5** A constrained wrap keeps its side site only where that brings the path closer to OpenSim's (iliacus and psoas
  went the long way round the pelvic brim, up to 180 mm).

MyoConverter's own moment-arm step (Step 2) didn't run for any muscle of the two gait models: it only moves wrap
sites, and they have none. Details: {repo}`station3/FINDINGS.md`.

## Muscle forces (Station 4)

- **4.1 Deterministic refit.** After MyoConverter's particle swarm, each muscle is refitted by least squares against
  OpenSim's force at four activation levels (MyoConverter fitted full activation only: mean errors 10–16% of max force,
  up to 121%).
- **4.2** Only poses where OpenSim has a static equilibrium are used.
- **4.3** The result doesn't depend on the swarm's luck, and left and right muscles get the same parameters
  (MyoConverter's swarm is unseeded: its copies of one muscle came out 1.5% and 7.9% off).
- **4.4 Both ends of the activation range.** A rigid tendon can't match an elastic one everywhere; the fit keeps full
  activation and passive within 0.25% of their own best fits and lets the middle pay.

Details: {repo}`station4/FINDINGS.md`.

## Walking (Station 5)

- **5.1 Robust path fit.** OpenSim's on/off jumps no longer pull a conditional point's fit away everywhere else.
- **5.2 Time step.** Models are saved with a 2 ms time step (MyoConverter saved the 5 ms its analysis last set, which
  made forward simulation inaccurate).
- **5.3** A conditional point next to a fixed one can switch toward it through a helper joint.
- **5.4** The force refit leaves out the few poses where the path itself is wrong.

Details: {repo}`station5/FINDINGS.md`.

## Past MyoConverter's limitations (Station 6)

| Change | What |
| --- | --- |
| 6.1 | Read every OpenSim 4 file: `::` in element names, coordinates that omit `<range>`/`<clamped>`/`<locked>`, mesh extensions in any case, placeholder meshes, joints whose parent socket names a body |
| 6.2 | Planar, Free, Ball and Gimbal joints |
| 6.3 | Mass properties exactly OpenSim's (the template's compiler bounds inflated small bodies) |
| 6.4 | CoordinateLimitForce |
| 6.5 | PathActuator and PathSpring |
| 6.6 | Contact spheres, half-spaces and Hunt–Crossley forces, with calibrated contact parameters |
| 6.7 | ASCII STL and OBJ meshes |
| 6.8 | A half-space on a moving body becomes a box |
| 6.9 | The contact calibration handles coupled coordinates and locked platforms |
| 6.10 | No damping on joints a coupling drives (it made Rajagopal's kneecap lag the knee by 1.9°) |
| 6.11 | A mesh file named in another case is found |
| 6.12 | Contact parameters must keep the whole model stable (the best fit threw a free walker model 4 m into the air) |
| 6.13 | The calibration's OpenSim simulations run in a fresh process (they failed now and then inside the conversion) |

Details: {repo}`station6/FINDINGS.md`.

## The tool around it

MyoConverter is a conversion pipeline. osim2mujoco adds:

- the support report (`osim2mujoco check`), which predicts whether and how a model converts before converting it;
- the validation report, with the checks used throughout this project;
- a `convert` command that refuses an output folder holding old results (MyoConverter's pipeline would silently
  resume from them) and a model it can't convert, before doing anything.
