# What converts

`osim2mujoco check model.osim` gives this for one model, element by element. This page is the overview. Change
numbers (like 6.2) refer to {doc}`differences`.

## Bodies and geometry

| OpenSim | MuJoCo | Notes |
| --- | --- | --- |
| Body: mass, centre of mass, inertia | body with exactly the same values | MyoConverter's template raised every inertia below 0.001 kg·m² (6.3) |
| Mesh (`.vtp`, `.stl`, `.obj`, any case) | mesh geom | `.vtp` becomes binary STL; ASCII STL is rewritten as binary (6.7); a file named in another case is found (6.11) |
| Placeholder mesh (`Unassigned`) | nothing | OpenSim shows nothing either (6.1) |
| Other attached geometry (bricks, cylinders, ...) | not converted | listed as ignored; visual only |
| Geometry on offset frames inside a body | not converted | listed as ignored; visual only |
| Gravity | the model's gravity | MyoConverter used MuJoCo's default (1c.1) |
| `<defaults>` | applied as OpenSim does | MyoConverter ignored it (1c.3) |

Only bodies reached from ground through the joints are converted. None of the 56 model files OpenSim distributes has
one that isn't.

## Joints and coordinates

| OpenSim | MuJoCo | Notes |
| --- | --- | --- |
| PinJoint, SliderJoint, UniversalJoint, WeldJoint | the equivalent joints | |
| CustomJoint, linear axes | hinge or slide joints | |
| CustomJoint, spline axes (a kneecap following the knee) | a quartic coupling | fitted over the coordinate's range (2.3); where one quartic is off by more than 0.25° or 0.5 mm, two in sequence through a helper joint (2.4) |
| CustomJoint, polynomial axes (degree ≤ 4) | the same polynomial, exactly | 2.5; RajagopalLaiUhlrich2023's knee |
| PlanarJoint, FreeJoint, BallJoint, GimbalJoint | the CustomJoint each one is, exactly | 6.2 |
| Coordinate range, clamped, locked, default value | joint range, limited, lock, `default-pose` keyframe | omitted properties take OpenSim's defaults (6.1) |
| CoordinateCouplerConstraint | joint coupling | splines fitted, linear functions exact |
| Polynomials of degree > 4, couplers driven by more than one coordinate | **unsupported** | the support report says so |
| PointOnLineConstraint | **unsupported** | BouncingBlock, the two OpenSim files that don't convert |

## Muscles and paths

| OpenSim | MuJoCo | Notes |
| --- | --- | --- |
| Thelen2003, Millard2012Equilibrium, Schutte1993 muscles | MuJoCo muscle | rigid tendon; force–length parameters fitted to OpenSim's static force at four activation levels (4.1–4.4) |
| PathPoint | site | |
| MovingPathPoint | a point on slide joints driven by its coordinate | fitted over the coordinate's range (3.2, 3.3) |
| ConditionalPathPoint | a moving point fitted to OpenSim's path | approximated: OpenSim switches it on and off, MuJoCo can't (3.1, 5.1, 5.3) |
| WrapCylinder, WrapSphere | the same wrap | wraps take the same side of the object as OpenSim's (3.4, 3.5) |
| WrapEllipsoid | a cylinder or sphere | approximated |
| WrapTorus | a small sphere the path passes through | approximated |
| Ligament | a tendon that exerts no force | approximated: its stiffness and force–length curve are dropped |

Not converted: tendon compliance (MuJoCo's tendons are rigid), fiber damping, force–velocity parameters. See
{doc}`limitations`.

## Other forces

| OpenSim | MuJoCo | Notes |
| --- | --- | --- |
| CoordinateActuator | motor with gear = optimal force | MyoConverter dropped the strength (1c.2) |
| CoordinateLimitForce | dead-band springs on the coordinate | exact beyond the limit, within 1% of peak inside the transition; the damping isn't converted (6.4) |
| PathActuator | tendon plus actuator | 6.5 |
| PathSpring | tendon with a one-sided spring | exact at zero speed; dissipation not converted (6.5) |
| BushingForce, FunctionBasedBushingForce, PointActuator, TorqueActuator, SpringGeneralizedForce | **skipped** | with a warning |

## Contact

| OpenSim | MuJoCo | Notes |
| --- | --- | --- |
| ContactSphere | sphere geom | exact position (6.6) |
| ContactHalfSpace | plane, or a large box on a body that can move | MuJoCo allows planes only on static bodies (6.8) |
| HuntCrossleyForce | contact pairs | geometry exact; stiffness, damping and friction fitted to OpenSim drops of the model, and checked to keep the whole model stable (6.6, 6.9, 6.12, 6.13) |
| Contact meshes, elastic foundation | **not converted** | |

When a model has its own contact, no ground plane is added. Otherwise a ground plane is added, unless you pass
`--no-ground`.

## Markers

Markers become sites named `<marker>_marker` on the same body.
