# Support report: RajagopalLaiUhlrich2023

Predicted outcome of the XML conversion step: **fails** (first failure: TransformAxis `walker_knee_r/translation1`)

| Status | Count |
| --- | --- |
| converted | 652 |
| approximated | 104 |
| skipped | 0 |
| ignored | 12 |
| unsupported | 2 |

| Status | Section | Type | Element | Reason |
| --- | --- | --- | --- | --- |
| unsupported | JointSet | TransformAxis | `walker_knee_l/translation1` | joints/CustomJoint.py _designate_dof: the first axis of each coordinate (processing order translation1-3, rotation1-3) must be SimmSpline or LinearFunction -> NotImplementedError (first axis of coordinate knee_angle_l uses PolynomialFunction) |
| unsupported | JointSet | TransformAxis | `walker_knee_r/translation1` | joints/CustomJoint.py _designate_dof: the first axis of each coordinate (processing order translation1-3, rotation1-3) must be SimmSpline or LinearFunction -> NotImplementedError (first axis of coordinate knee_angle_r uses PolynomialFunction) |
| ignored | JointSet | TransformAxis | `walker_knee_r/translation2`, `walker_knee_r/translation3`, `walker_knee_r/rotation1` (+7 more) | not processed: conversion stops at an earlier axis of this joint |
| ignored | Model | ContactGeometrySet | `ContactGeometrySet` | converter.convert: this top-level set/component is never read (empty) |
| ignored | Model | ControllerSet | `ControllerSet` | converter.convert: this top-level set/component is never read (empty) |
| approximated | ForceSet | CoordinateActuator | `lumbar_ext`, `lumbar_bend`, `lumbar_rot` (+14 more) | forces/CoordinateActuator.py: motor on the joint; optimal_force read but dropped (upstream TODO), so torque scaling differs |
| approximated | ForceSet | Millard2012EquilibriumMuscle | `addbrev_r`, `addlong_r`, `addmagDist_r` (+77 more) | forces/Muscle.py: rigid-tendon MuJoCo muscle; F-L params fitted in Step 3; activation time constants kept; tendon compliance, fiber damping and F-V parameters not converted |
| approximated | JointSet | TransformAxis | `patellofemoral_r/translation1`, `patellofemoral_r/translation2`, `patellofemoral_r/rotation1` (+3 more) | joints/CustomJoint.py: SimmSpline/NaturalCubicSpline -> quartic polynomial (fitted over the independent coordinate's range, Station 2 change 2.3) + joint equality |
| approximated | Model | gravity | `gravity` | OpenSim gravity (0 -9.8066499999999994 0) not converted; MuJoCo default (0 0 -9.81) used |
| converted | — | — | Body ×22, Mesh ×81, WrapCylinder ×44, CoordinateCouplerConstraint ×2, PathPoint ×288, PathWrap ×46, Coordinate ×39, CustomJoint ×10, PinJoint ×10, TransformAxis ×42, UniversalJoint ×2, Marker ×66 | — |
