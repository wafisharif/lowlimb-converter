# What converts

Run `osim2mujoco check model.osim` for a specific model. This page is the general picture.

## Bodies and geometry

| OpenSim | MuJoCo | Notes |
| --- | --- | --- |
| Body mass, centre of mass, inertia | same values, exactly | |
| Meshes (`.vtp`, `.stl`, `.obj`) | mesh geoms | `.vtp` and ASCII STL become binary STL; file-name case doesn't matter |
| `Unassigned` placeholder meshes | nothing | |
| Other attached geometry, geometry on offset frames | not converted | visual only |
| Gravity, `<defaults>` | applied as in OpenSim | |

Only bodies connected to ground through joints are converted. None of OpenSim's 56 example models has any that
aren't.

## Joints

| OpenSim | MuJoCo | Notes |
| --- | --- | --- |
| Pin, Slider, Universal, Weld | same joints | |
| CustomJoint, linear axes | hinge or slide joints | |
| CustomJoint, spline axes | quartic coupling | fitted over the coordinate's range; two chained quartics when one is off by more than 0.25° or 0.5 mm |
| CustomJoint, polynomial axes up to degree 4 | the same polynomial | exact |
| Planar, Free, Ball, Gimbal | equivalent CustomJoint | exact |
| Coordinate range, clamped, locked, default value | range, limited, lock, `default-pose` keyframe | missing properties take OpenSim's defaults |
| CoordinateCouplerConstraint | joint coupling | splines fitted, linear functions exact |
| Polynomials above degree 4, couplers with several inputs, PointOnLineConstraint | unsupported | |

## Muscles

| OpenSim | MuJoCo | Notes |
| --- | --- | --- |
| Thelen2003, Millard2012Equilibrium, Schutte1993 | MuJoCo muscle | rigid tendon; fitted to OpenSim's static force at four activation levels |
| PathPoint | site | |
| MovingPathPoint | point on slide joints driven by its coordinate | fitted over the coordinate's range |
| ConditionalPathPoint | moving point fitted to OpenSim's path | approximated; MuJoCo can't switch points on and off |
| WrapCylinder, WrapSphere | same wrap | wraps on the same side as in OpenSim |
| WrapEllipsoid | cylinder or sphere | approximated |
| WrapTorus | small sphere the path goes through | approximated |
| Ligament | tendon with no force | stiffness and force–length curve dropped |

Tendon compliance, fiber damping and force–velocity parameters aren't converted.

## Other forces

| OpenSim | MuJoCo | Notes |
| --- | --- | --- |
| CoordinateActuator | motor, gear = optimal force | |
| CoordinateLimitForce | dead-band springs on the coordinate | within 1% of peak; damping not converted |
| PathActuator | tendon and actuator | |
| PathSpring | tendon with a one-sided spring | dissipation not converted |
| Bushings, PointActuator, TorqueActuator, SpringGeneralizedForce | skipped, with a warning | |

## Contact

| OpenSim | MuJoCo | Notes |
| --- | --- | --- |
| ContactSphere | sphere geom | |
| ContactHalfSpace | plane, or a large box if it's on a moving body | MuJoCo only allows planes on static bodies |
| HuntCrossleyForce | contact pairs | stiffness, damping and friction fitted to OpenSim drops of the model |
| Contact meshes, elastic foundation | not converted | |

A ground plane is added unless the model has its own contact or you pass `--no-ground`.

Markers become sites named `<marker>_marker`.
