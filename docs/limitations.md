# Limitations

## Open problems

**Contact dynamics don't generalise (Station 6, gate L7).** The contact geometry is exact, but the physics of contact
isn't. OpenSim's Hunt–Crossley contact is a nonlinear spring (force grows with penetration to the power 1.5) with
speed-dependent damping and a smooth friction law. MuJoCo's soft contact is a linear spring–damper with Coulomb
friction. The converter fits MuJoCo's stiffness, damping and friction to two vertical drops of the model in OpenSim.
On OpenSim's WalkerModel, checked on drops it never saw:

| Drop | osim2mujoco | MuJoCo's default parameters |
| --- | --- | --- |
| straight down from 0.05 m | **8.0 mm** | 16.2 mm |
| tilted +0.15 rad | **37.5 mm** | 43.0 mm |
| tilted −0.15 rad | 61.8 mm | **13.6 mm** |
| sliding at 0.5 m/s | 56.8 mm | **11.4 mm** |

(RMS error of the pelvis position over 1 s.) Treat the converted contact parameters as a starting point, and check
them against OpenSim for the motion you care about. Fixing this needs a contact model closer to Hunt–Crossley, and
calibration drops that include rocking and sliding.

**Rigid tendons.** MuJoCo's muscles have no elastic tendon. A rigid tendon can't follow how an elastic one shifts the
force–length curve with activation, so the force fit keeps full activation and passive closest and lets the middle of
the range pay. How force changes with speed (force–velocity) isn't checked at all.

**Intermittent CI failure.** On one run of GitHub's Linux machine, two models (SoccerKick, Leg39) failed to convert
inside a test, and passed on the next run. The cause isn't known yet. The test now prints the error if it happens
again.

## MyoConverter's documented limitations

MyoConverter's documentation lists ten limitations. Where osim2mujoco stands on each:

| # | MyoConverter's limitation | Status |
| --- | --- | --- |
| 1 | Only one kinematic tree | Unchanged, but detected: the support report lists any body not reached from ground. None of OpenSim's 56 model files has one. |
| 2 | Contacts not converted (except a ground plane) | **Geometry fixed** for spheres and half-spaces; dynamics open (above). Contact meshes and elastic foundations are still not converted. |
| 3 | `<defaults>` ignored | **Fixed** (1c.3) |
| 4 | Conditional and moving path points may move tendons inaccurately | **Fixed**: both are fitted to OpenSim's own path (worst moment-arm error 69–78 mm → 3.5–6.3 mm) |
| 5 | Missing parsers: bushings, CoordinateLimitForce, PointActuator, TorqueActuator | **Mostly fixed**: CoordinateLimitForce, PathActuator, PathSpring convert; bushings, PointActuator and TorqueActuator are still skipped with a warning (no OpenSim example model uses the last two) |
| 6 | CustomJoint: non-linear dependencies | **Mostly fixed**: spline axes fitted (and chained where one quartic isn't enough), polynomial axes up to degree 4 exact, four more joint types. Still unsupported: polynomials above degree 4, couplers driven by more than one coordinate, PointOnLineConstraint |
| 7 | Wrapping ellipsoids become cylinders or spheres; wrap orientation not optimised | Wrap sides fixed (3.4, 3.5). Ellipsoids are still approximated (no supported model has one). A path that wraps over two objects in a row keeps one: MuJoCo needs a site between two wraps (Rajagopal 2016's gastrocnemius, 13 mm RMS at worst) |
| 8 | Moment-arm jumps where a path crosses a wrap object aren't smoothed | Measured: every pair's worst single error is reported |
| 9 | Muscle operating range assumed to be in [0, 1.8] | **Removed** (4.1) |
| 10 | Rigid tendons | Unchanged (above); quantified: mean force error at full activation 2.5–3.6% of max force |

The full table, with the evidence for each row: {repo}`LIMITATIONS.md`.

## Other things that aren't converted

- Geometry other than meshes (bricks, cylinders and so on) and geometry on offset frames inside a body: visual only.
- Muscle fiber damping and force–velocity parameters; Ligament stiffness.
- CoordinateLimitForce damping and PathSpring dissipation.
- Prescribed coordinate motion.

`osim2mujoco check` lists everything a particular model has that isn't converted.
