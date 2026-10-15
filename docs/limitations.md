# Limitations

## Contact dynamics

Contact geometry converts exactly; contact behaviour doesn't. OpenSim's Hunt–Crossley contact is a nonlinear spring
(force grows with penetration^1.5) with speed-dependent damping and smooth friction. MuJoCo's is a linear
spring–damper with Coulomb friction. The converter fits MuJoCo's stiffness, damping and friction to two vertical drops
of the model in OpenSim, which doesn't carry over to other motions. On WalkerModel, using drops the fit never saw
(pelvis position RMS error over 1 s):

| Drop | osim2mujoco | MuJoCo defaults |
| --- | --- | --- |
| Straight down from 0.05 m | 8.0 mm | 16.2 mm |
| Tilted +0.15 rad | 37.5 mm | 43.0 mm |
| Tilted −0.15 rad | 61.8 mm | 13.6 mm |
| Sliding at 0.5 m/s | 56.8 mm | 11.4 mm |

Use the fitted parameters as a starting point and check them against OpenSim for your own motion. A real fix needs a
contact model closer to Hunt–Crossley and fitting drops that include rocking and sliding.

## Rigid tendons

MuJoCo muscles don't have elastic tendons. A rigid tendon can't reproduce how an elastic one shifts the force–length
curve with activation, so the force fit favours full activation and passive. Force–velocity behaviour isn't checked.

## MyoConverter's listed limitations

| # | Limitation | Status |
| --- | --- | --- |
| 1 | One kinematic tree only | Unchanged, but `check` reports bodies not connected to ground. None of OpenSim's example models has any. |
| 2 | No contact except a ground plane | Spheres and half-spaces convert with exact geometry; dynamics as above. Contact meshes and elastic foundations still don't. |
| 3 | `<defaults>` ignored | Fixed |
| 4 | Conditional and moving path points inaccurate | Fixed: fitted to OpenSim's path. Worst moment-arm error went from 69–78 mm to 3.5–6.3 mm. |
| 5 | No bushings, CoordinateLimitForce, PointActuator, TorqueActuator | CoordinateLimitForce now converts (as do PathActuator and PathSpring). The others are still skipped; no OpenSim example uses PointActuator or TorqueActuator. |
| 6 | CustomJoint non-linear dependencies | Splines are fitted, polynomials up to degree 4 are exact, and four more joint types convert. Higher-degree polynomials, multi-input couplers and PointOnLineConstraint are still unsupported. |
| 7 | Ellipsoid wraps approximated, wrap side not optimised | Wrap sides fixed. Ellipsoids are still approximated. A path over two wraps in a row keeps only one, since MuJoCo needs a site between them (Rajagopal 2016's gastrocnemius: 13 mm RMS). |
| 8 | Moment-arm jumps at wraps not smoothed | Not changed; the worst single error per pair is reported |
| 9 | Muscle operating range assumed within [0, 1.8] | Removed |
| 10 | Rigid tendons | Unchanged; force error at full activation is 2.5–3.6% of max force |

Evidence for each row: {repo}`LIMITATIONS.md`.

## Also not converted

- Non-mesh geometry and geometry on offset frames (visual only)
- Fiber damping, force–velocity parameters, ligament stiffness
- CoordinateLimitForce damping, PathSpring dissipation
- Prescribed coordinate motion
