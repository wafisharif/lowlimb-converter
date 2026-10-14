# MyoConverter's documented limitations, and where this fork stands

MyoConverter lists ten limitations in its documentation
([myoconverter.readthedocs.io, "Limitations"](https://myoconverter.readthedocs.io/en/latest/limitations.html)).
For each one: what this fork changed, and the evidence (a station's gated result or a test). "Supported models" means
the four models checked end to end (Stations 2–5): gait10dof18musc, gait2354, Rajagopal2016 and
RajagopalLaiUhlrich2023 (Station 2 only). Station 6 also runs the XML step on all 56 model files OpenSim distributes
(opensim-models @ d9b05d4).

| # | MyoConverter's limitation | Status here | Evidence |
| --- | --- | --- | --- |
| 1 | Only one kinematic tree | Unchanged, but detected | The support report (`lowlimb-convert check`) lists every body not reached from ground as ignored before converting. None of the 56 files OpenSim distributes has one (support report, Station 6) |
| 2 | Contacts not converted (except a ground plane) | **Geometry fixed for spheres and half-spaces; dynamics open** (changes 6.6, 6.8, 6.9, 6.12, 6.13) | `ContactSphere`, `ContactHalfSpace` and `HuntCrossleyForce` become MuJoCo geoms and contact pairs with exactly OpenSim's geometry (Station 6 gate L6: signed distance within 1e-6 m in every pose), and the pair parameters are calibrated against OpenSim drops of the model. The calibrated parameters keep the whole model stable (6.12), but they don't generalise: on drops it never saw, OpenSim's WalkerModel is followed within 8 mm from another height, and only within 37–62 mm when tilted or sliding, worse than MuJoCo's default contact parameters on two of three (gate L7 fails; open item, `station6/FINDINGS.md`). A half-space on a moving body becomes a box (MuJoCo allows planes only on static bodies). Other contact geometry (meshes, elastic foundation) is still listed as ignored |
| 3 | The `<defaults>` element is ignored | **Fixed** (change 1c.3) | A component that omits a property now takes it from `<defaults>`, as OpenSim does. `station1/tests/test_document_defaults.py` (OpenSim reads the default; the converted actuator strength, control range and muscle peak force match it; explicit values still win). In Rajagopal 2016 this sets every muscle's lowest control to OpenSim's 0.01 |
| 4 | Conditional and moving path points may move tendons inaccurately | **Fixed** (changes 3.1–3.3, 5.1, 5.3) | Both point types are fitted to OpenSim's own path. Worst moment-arm error 69–78 mm → 3.5–6.3 mm over random poses (Station 3); muscle lengths along a real walking trial 33–42× closer than MyoConverter's (Station 5) |
| 5 | Parsers missing: BushingForce, FunctionBasedBushingForce, CoordinateLimitForce, PointActuator, TorqueActuator | **Mostly fixed** (changes 1c.2, 6.4, 6.5) | `CoordinateLimitForce` becomes dead-band springs on the coordinate, exact beyond OpenSim's transition and within 1% of its peak force inside it (gate L4; its damping isn't converted and is reported). `PathActuator` and `PathSpring` (not in MyoConverter's list, but used by OpenSim's exoskeleton examples) convert with exact paths (gate L5). CoordinateActuators get their strength (change 1c.2). Still skipped, with a warning and listed by the support report: bushings, PointActuator, TorqueActuator (no OpenSim example model uses the last two) |
| 6 | CustomJoint: non-linear dependencies not supported | **Mostly fixed** (changes 2.3–2.5, 6.2) | Spline axes are fitted over the coordinate's range (2.3); where one quartic can't follow a spline, two run in sequence through a helper joint (2.4: Rajagopal's patella 0.64° → 0.04°); PolynomialFunction axes up to degree 4 convert exactly (2.5: RajagopalLaiUhlrich2023's knee, which MyoConverter couldn't convert at all). Planar, Free, Ball and Gimbal joints convert as the CustomJoint each one is, exactly (6.2). Still unsupported: polynomials above degree 4, non-linear coupler constraints between coordinates, PointOnLineConstraint |
| 7 | Wrapping ellipsoids become cylinders or spheres; wrap orientation isn't optimised | Measured; wrap sides fixed (changes 3.4, 3.5) | No supported model has an ellipsoid. MyoConverter forced every wrap onto one side of its object; an unconstrained object (OpenSim quadrant "all") now wraps on whichever side is shorter (3.4: Rajagopal's vasti 127 mm → 0.03 mm length error), and a constrained one keeps its side site only where that brings the path closer to OpenSim's (3.5: iliacus and psoas went the long way round the pelvic brim, up to 180 mm too long; now within 0.03 mm). Rajagopal 2016's moment arms: 0.46 mm mean RMS error vs MyoConverter's 1.64 mm (Station 3). A path segment that wraps over two objects in a row keeps one (MuJoCo needs a site between two wrapping geoms): the gastrocnemii, worst pair 13 mm RMS |
| 8 | Moment-arm jumps (a path crossing a wrap object) not smoothed | Measured | Station 3 reports every pair's worst single error |
| 9 | Muscle operating range assumed to lie in [0, 1.8] | **Removed** (change 4.1) | The refit allows ranges in [−3, 6]; 4 of 18 (gait10dof18musc) and 10 of 54 (gait2354) muscles use values outside [0, 1.8] (Station 4) |
| 10 | Rigid (non-elastic) tendons | Unchanged; quantified | MuJoCo still has no elastic-tendon muscle. A rigid tendon can't follow how an elastic one shifts the force–length curve with activation, so the static-force fit keeps the two ends (full activation and passive) close to their own best fits and lets the middle pay (change 4.4). Static force errors, mean over muscles at full activation: 3.1% (gait10dof18musc), 3.7% (gait2354), 2.5% (Rajagopal 2016; MyoConverter 6.9%) of max isometric force; force–velocity behaviour is not validated (Stations 4–5) |

What this fork adds that MyoConverter's documentation doesn't mention, each found by a station's check:
- MyoConverter can't read most of OpenSim's own example models: element names like
  `HuntCrossleyForce::ContactParametersSet` (lxml refuses them), coordinates without `<clamped>`/`<locked>`, `.STL` in
  upper case, ASCII STL files, mesh file names that differ from the file's only in case, placeholder meshes and joints
  whose parent socket names a body all stopped its XML step. This fork reads all of them, and OBJ meshes too (changes
  6.1, 6.7, 6.11); of the 56 files OpenSim distributes, 54 convert (MyoConverter: 40), and the
  support report predicts every file's outcome before converting (Station 6, gates L0–L1).
- Small bodies were made heavier: the template's compiler bounds raised every inertia below 0.001 kg m² (the gait
  models' toes, 0.0001–0.0002 → 0.001). Mass properties now equal OpenSim's exactly (change 6.3, gate L3).
- MyoConverter's moment-arm optimisation (Step 2) never ran for any muscle of the two gait models it ships: it only
  moves wrapping-object sites, and these models have none (Station 0).
- Joints with an offset child frame rotated about the wrong point (Rajagopal 2016's knee 15.6 mm off), numbers were
  written with 4 significant figures, and gravity was not converted (Stations 1b–2).
- Step 3's particle swarm is unseeded, so a conversion's muscle parameters change from run to run and left and right
  copies of a muscle differ (in its Rajagopal 2016 conversion, vastus lateralis is 1.5% of Fmax off on one side and
  7.9% on the other); the refit is deterministic and symmetric (Station 4).
- Converted models were saved with the time step MyoConverter's analysis last set (5 ms instead of 2 ms), which made
  forward simulation of the gait models wildly inaccurate (Station 5).
- Rerunning the pipeline into a folder with old results silently reuses them (Station 3); the tool refuses such a
  folder.
- MyoConverter validates only static curves. This fork also checks a real walking trial: muscle lengths and moment arms
  over the gait cycle, and the joint moments needed to walk with the measured ground forces (Station 5), available to
  any user through `lowlimb-convert validate --ik ... --grf ...`; and contact, limit forces, path actuators and mass
  properties (the "elements" level, Station 6).
