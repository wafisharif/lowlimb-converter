# Support report: Rajagopal2016

Predicted outcome of the XML conversion step: **converts**

| Status | Count |
| --- | --- |
| converted | 670 |
| approximated | 94 |
| skipped | 0 |
| ignored | 2 |
| unsupported | 0 |

| Status | Section | Type | Element | Reason |
| --- | --- | --- | --- | --- |
| ignored | Model | ContactGeometrySet | `ContactGeometrySet` | converter.convert: this top-level set/component is never read (empty) |
| ignored | Model | ControllerSet | `ControllerSet` | converter.convert: this top-level set/component is never read (empty) |
| approximated | ForceSet | Millard2012EquilibriumMuscle | `addbrev_r`, `addlong_r`, `addmagDist_r` (+77 more) | forces/Muscle.py: rigid-tendon MuJoCo muscle; F-L params fitted in Step 3; activation time constants kept; tendon compliance, fiber damping and F-V parameters not converted |
| approximated | JointSet | TransformAxis | `walker_knee_r/translation1`, `walker_knee_r/translation2`, `walker_knee_r/rotation2` (+11 more) | joints/CustomJoint.py: SimmSpline/NaturalCubicSpline -> quartic polynomial (fitted over the independent coordinate's range, Station 2 change 2.3) + joint equality; where one quartic is off by more than 0.25 deg / 0.5 mm, two chained quartics through a helper joint (change 2.4) |
| converted | — | — | Body ×22, Mesh ×81, WrapCylinder ×40, CoordinateCouplerConstraint ×2, CoordinateActuator ×17, PathPoint ×288, PathWrap ×46, Coordinate ×39, CustomJoint ×10, PinJoint ×10, TransformAxis ×46, UniversalJoint ×2, Marker ×66, gravity ×1 | — |
