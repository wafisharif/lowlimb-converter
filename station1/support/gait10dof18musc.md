# Support report: gait10dof18musc.osim

Predicted outcome of the XML conversion step: **converts**

| Status | Count |
| --- | --- |
| converted | 115 |
| approximated | 35 |
| skipped | 0 |
| ignored | 4 |
| unsupported | 0 |

| Status | Section | Type | Element | Reason |
| --- | --- | --- | --- | --- |
| ignored | Model | ComponentSet | `ComponentSet` | converter.convert: this top-level set/component is never read (empty) |
| ignored | Model | ContactGeometrySet | `ContactGeometrySet` | converter.convert: this top-level set/component is never read (empty) |
| ignored | Model | ControllerSet | `ControllerSet` | converter.convert: this top-level set/component is never read (empty) |
| ignored | Model | ProbeSet | `ProbeSet` | converter.convert: this top-level set/component is never read (empty) |
| approximated | ForceSet | ConditionalPathPoint | `iliopsoas_r/psoas_r-P3`, `vasti_r/vas_int_r-P3`, `gastroc_r/med_gas_r-P2` (+3 more) | path_points/ConditionalPathPoint.py: modelled as a moving point anchored to the nearest plain PathPoint on the same body (step change approximated by a polynomial) |
| approximated | ForceSet | ConditionalPathPoint | `rect_fem_r/rect_fem_r-P2`, `rect_fem_l/rect_fem_l-P2` | path_points/ConditionalPathPoint.py: no anchor PathPoint on the same body -> warning, treated as a normal fixed PathPoint (conditional behaviour lost) |
| approximated | ForceSet | Millard2012EquilibriumMuscle | `hamstrings_r`, `bifemsh_r`, `glut_max_r` (+15 more) | forces/Muscle.py: rigid-tendon MuJoCo muscle; F-L params fitted in Step 3; activation time constants kept; tendon compliance, fiber damping and F-V parameters not converted |
| approximated | ForceSet | MovingPathPoint | `rect_fem_r/rect_fem_r-P3`, `vasti_r/vas_int_r-P4`, `rect_fem_l/rect_fem_l-P3` (+1 more) | path_points/MovingPathPoint.py: child body with slide joints; each axis fitted with a quartic polynomial of its coordinate (path_points/utils.get_moving_path_point_dependency) |
| approximated | JointSet | TransformAxis | `knee_r/translation1`, `knee_r/translation2`, `knee_l/translation1` (+1 more) | joints/CustomJoint.py: SimmSpline/NaturalCubicSpline -> quartic polynomial + joint equality |
| approximated | Model | gravity | `gravity` | OpenSim gravity (0 -9.8066499999999994 0) not converted; MuJoCo default (0 0 -9.81) used |
| converted | — | — | Body ×12, Mesh ×19, PathPoint ×48, Coordinate ×10, CustomJoint ×3, PinJoint ×5, TransformAxis ×14, WeldJoint ×4 | — |
