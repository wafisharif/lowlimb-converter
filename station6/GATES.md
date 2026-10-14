# Station 6 gates: past MyoConverter's limitations

Written 2026-10-12, before any Station 6 code existed or had run. A copy was posted to the plan doc (version history
timestamps it) before any run.

**Question:** MyoConverter's documentation lists limitations
([limitations page](https://myoconverter.readthedocs.io/en/latest/limitations.html)): contacts aren't converted;
CoordinateLimitForce, TorqueActuator, PointActuator, bushings and other forces have no parser; non-linear coordinate
dependencies aren't supported; conditional and moving path points are inaccurate (fixed in Stations 3 and 5);
tendons are rigid; and more. Which of these can be removed for the models OpenSim itself distributes, and are the new
conversions checked as strictly as the old ones? (Already done before this station: `<defaults>` (change 1c.3) and
PolynomialFunction axes (change 2.5, RajagopalLaiUhlrich2023 at Station 2); conditional and moving path points
(Stations 3 and 5); see `LIMITATIONS.md`.)

Before writing these gates we surveyed the 56 `.osim` files in opensim-models @ d9b05d4 with the support report
(predictions only, no conversion, no comparison). Besides the documented limitations, three more reasons stop the
XML step on OpenSim's own files: element names like `HuntCrossleyForce::ContactParametersSet` (OpenSim writes them;
lxml refuses them, so the file can't even be read), coordinates that leave out `<clamped>` or `<locked>` (OpenSim
uses defaults; the converter crashes), and mesh files named `.STL` (upper case) or `Unassigned`.

## Changes planned (each its own commit, with a test that fails on the converter before it)

| Change | What | Models it is for |
| --- | --- | --- |
| 6.1 | Read every OpenSim 4 file: `::` element names, missing coordinate properties (OpenSim's class defaults, when `<defaults>` doesn't give them either), mesh extensions in any case, placeholder meshes (`Unassigned`, OpenSim shows nothing) skipped with a warning | WalkerModel ×2, ToyLanding ×3, DynamicJumper, SoccerKick, BouncingBlock ×2, Pendulum |
| 6.2 | `PlanarJoint`, `FreeJoint`, `BallJoint`, `GimbalJoint`, converted as the CustomJoint each one is | WalkerModel ×2, SoccerKick, BouncingBlock (synthetic tests for Ball, Gimbal) |
| 6.3 | Mass properties exactly as OpenSim's: no inflation of small inertias by the template's compiler bounds | every model (found while planning: the gait models' toes, 0.0001 → 0.001 kg m²) |
| 6.4 | `CoordinateLimitForce` → springs on the coordinate (MuJoCo fixed tendons with dead-band springs) | 13 CLFs in 6 files |
| 6.5 | `PathActuator` (tendon + motor) and `PathSpring` (tendon with a one-sided spring) | gait10 subject01 exoskeleton variants ×4 |
| 6.6 | Contact: `ContactSphere`, `ContactHalfSpace` and `HuntCrossleyForce` → MuJoCo geoms and contact pairs, with exact geometry and contact parameters calibrated against OpenSim simulations | WalkerModel ×2, ToyLanding ×3, DynamicJumper, SoccerKick |

Out of scope: `PointOnLineConstraint` and `BushingForce` (BouncingBlock, ToyLanding AFOs; reported as before),
`TorqueActuator`/`PointActuator`/`SpringGeneralizedForce` (no file uses them outside `<defaults>`), ellipsoid wrap
orientation, elastic tendons (no native MuJoCo support, `station0/elastic_tendon`), OpenSim 3 files.

## Columns

- **Ours:** the converter after each change (run scripts regenerate everything).
- **MyoConverter:** the Station 1a port (tag `station1a`: upstream MyoConverter @ cadf380 with only the four port
  fixes, on the same modern stack). Station 1a showed it produces the same models as upstream on MuJoCo 2.3.7.

## Model files and geometry

All 56 `.osim` files in opensim-models @ d9b05d4 (list and SHA-256 in `station6/models.tsv`, written by the run
script). Geometry folder per file, following OpenSim's own search order (model folder first, then its `Geometry/`,
then the shared one): the repo-wide `Geometry/`, overlaid with `<model folder>/Geometry/`, overlaid with mesh files
(`.vtp`, `.stl`, `.obj`, any case) in the model folder itself. Plus synthetic models built by `station6/tests` with
the OpenSim API (one free body on a Ball, Gimbal, Free and Planar joint, with offset and rotated joint frames).

## Gates

- **L0 THE SUPPORT REPORT IS RIGHT (all 56 files, both columns' converters):** for every file, the report's
  predicted outcome matches the real XML step (Station 1b's S3 matching: predicted "converts" → cvt1 written;
  predicted failure at element E → the converter stops at E). S1 (every element listed once, independent XPath
  counts) holds for every file. If L0 fails, the report is fixed before anything else is judged.
- **L1 COVERAGE:** the XML step converts every file except those whose first unsupported element is out of scope
  (above). Expected: 54 of 56 (only the two BouncingBlock files fail, at `PointOnLineConstraint`;
  RajagopalLaiUhlrich2023 already converts since change 2.5).
- **L2 KINEMATICS:** Station 2's K0–K2 (same method, same 1 mm / 0.5° limits, same code) pass for every model whose
  conversion uses a Station 6 joint change: WalkerModel, WalkerModelTerrain, SoccerKickingModel, and the four
  synthetic joint models; and still pass for gait10dof18musc, gait2354, Rajagopal2016 and RajagopalLaiUhlrich2023
  (regression). Also run on every other newly converting file (reported if not listed above).
- **L3 MASS PROPERTIES (every converted file):** for every OpenSim body with a valid inertia (positive definite,
  triangle inequality), MuJoCo's compiled mass, centre of mass and inertia tensor (about the centre of mass, body
  frame) equal OpenSim's within 1e-9 relative (1e-9 kg, 1e-9 m, 1e-9 × the largest principal moment). Bodies with
  an invalid OpenSim inertia are listed (reported).
- **L4 COORDINATE LIMIT FORCES (every CLF in a converted file):** with the speed at zero, sweep the CLF's coordinate
  (others at their defaults) over [lower − 3·transition − 10°, upper + 3·transition + 10°] (10 mm instead of 10° for
  translations) in 2001 steps. MuJoCo's generalized spring force on the joint from the CLF's elements must equal
  OpenSim's CLF generalized force within 1% of the largest |OpenSim force| over the sweep, at every step. MyoConverter
  drops CLFs, so its error is OpenSim's whole force (reported). Reported: the force at ±1 rad/s (±1 m/s) beyond each
  limit (damping).
- **L5 PATH ACTUATORS AND SPRINGS (the four exoskeleton files):** in Station 3's poses: tendon length vs OpenSim's
  path length RMS ≤ 1 mm and max ≤ 3 mm; generalized force on every coordinate the path crosses (PathActuator at
  control 1; PathSpring at zero speed) RMS error ≤ 2% of the largest |OpenSim generalized force| of that element.
- **L6 CONTACT GEOMETRY (every converted contact pair):** in Station 2's poses, the signed distance between the two
  geometries computed from OpenSim's frames equals MuJoCo's (`mj_geomDistance`) within 1e-6 m wherever OpenSim's
  is below 0.1 m; and OpenSim's HuntCrossleyForce is non-zero exactly where the pair penetrates (sign convention;
  poses within 1e-9 m of touching excluded).
- **L7 CONTACT DYNAMICS (WalkerModel, the one file with contact and no muscles or controllers):** all coordinates
  locked at their defaults except the pelvis's planar joint (OpenSim: locked coordinates; MuJoCo: the same rigid
  shape), the walker dropped onto its sloped platform. Heights h are above the height where the lowest contact sphere
  touches. Calibration (the converter may use these): h = 0.02 m and 0.10 m, at rest. **Held out** (never used for
  calibration or development): h = 0.05 m; h = 0.05 m with pelvis rotation +0.15 rad and −0.15 rad; h = 0.02 m with
  0.5 m/s horizontal (ground X) speed. 1.0 s each, compared every 1 ms (OpenSim's integrator at accuracy 1e-8).
  Error per run: RMS over time of the pelvis position difference (tx, ty; mm) and of its rotation (deg).
  - **C1:** every held-out run: position RMS ≤ 10 mm and rotation RMS ≤ 2°.
  - **C2:** mean held-out position RMS ≤ half of what the same MuJoCo model gets with MuJoCo's default contact
    parameters (solref 0.02 1, solimp 0.9 0.95 0.001 0.5 2, friction 1 1 0.005 0.0001 0.0001), i.e. what adding the
    contacts by hand gives (MyoConverter's suggestion).
  - Reported: the walker released unlocked from its default pose for 1 s (contacts, limit forces and gravity
    together); the other contact models' calibration results.

As in earlier stations, a model or element that misses a gate stays where it is: the cause is found and fixed as its
own change (logged, before/after), and the gates don't change. A miss may be listed with a reason only if a test
shows the reason.

## Expected from reading the code (before running)

- L1: 12 more files convert (54/56; RajagopalLaiUhlrich2023 already does). Some need several changes (e.g.
  ToyLanding needs 6.1 and 6.6).
- L2: Planar/Free/Ball/Gimbal as CustomJoints should be exact.
- L3: the MyoConverter column fails on every model with a body below 0.001 kg m² (both gait models' toes).
- L4: dead-band springs are exact outside OpenSim's smooth transition and approximate inside it; with several
  dead-band springs per limit fitted to OpenSim's onset curve, 1% should be reachable. CLF damping (active only
  beyond the limit) has no exact MuJoCo equivalent and is expected to be dropped or approximated (reported).
- L5: the PathSpring has a ConditionalPathPoint; change 3.1's fit only applies to muscles, so the upstream
  approximation may fail the length gate.
- L6: exact up to round-off.
- L7: unknown. MuJoCo's soft contact is a mass-normalised linear spring-damper; Hunt–Crossley is a nonlinear
  (x^1.5) force law. The walker's spheres sink about 0.04 mm under its weight, so the shape of the force law matters
  less than restitution and friction.

## Amendment 2026-10-13 (after runs 1 and 2 stopped at L0–L1, before run3)

What happened first (details in `EXPERIMENTS.md` and `station6/FINDINGS.md`): run1 failed L0 and L1 on five files
(ASCII STL meshes, a half-space on a body that can move, a rigid drop OpenSim couldn't assemble) and was stopped after
the coverage step, as the L0 rule says; changes 6.7–6.9 fixed them. run2 passed L0 (56/56) and failed L1 on Leg39 (a
mesh file named in a different case; change 6.11) and was stopped after the coverage step. A development run of
gates.py on run2's conversions then showed three problems in the checks themselves, below. **It also evaluated L7's
held-out drops**, which the gate says development never uses; that was a mistake in how the development run was set
up. Its L7 result (C1 and C2 FAIL) is reported as seen; the contact calibration was not changed after it.

Changes to the checks (gates and tolerances unchanged; every model already gated before is unaffected):
- **L6:** MuJoCo's distance is computed from the two MuJoCo geoms, each placed by its own offset in its MuJoCo body,
  with the bodies at OpenSim's body frames in that pose. As written, the check used MuJoCo's own kinematics, so on
  models whose knees are approximated within L2's limits (DynamicJumper, SoccerKick, ToyLanding) it measured those
  body errors (0.2–0.9 mm), not the contact geometry; WalkerModel's kinematics are exact, so its result is unchanged.
  MuJoCo's own kinematics are still reported. The sign convention is checked per force: a HuntCrossleyForce's record
  is the sum over its pairs, so it must be non-zero exactly where at least one of its pairs penetrates (the per-pair
  version could only pass forces with one pair).
- **L4:** every coordinate is unlocked in the check's copy of the OpenSim model, so a limit force on a coordinate the
  file locks (ToyLanding's left knee) is compared as a force law; OpenSim reported no force for it in the locked copy.
- **L2 and L6 (Station 2's harness, which poses MuJoCo):** an inactive joint equality is accepted when an active lock
  holds its joint (the converter deactivates the coupling of a coordinate whose independent coordinate is locked and
  locks the dependent joint instead), and K0's count of solver rows counts active equalities.
Seen before writing this: run1's and run2's L0–L1 results, and the development run's L2–L7 results on run2's
conversions (L2 PASS; L3 PASS; L4 FAIL on ToyLanding ×3; L5 PASS; L6 FAIL on DynamicJumper, SoccerKick, ToyLanding ×3;
L7 C1 and C2 FAIL).
