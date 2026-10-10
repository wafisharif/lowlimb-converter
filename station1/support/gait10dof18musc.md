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
| approximated | ForceSet | ConditionalPathPoint | `iliopsoas_r/psoas_r-P3`, `rect_fem_r/rect_fem_r-P2`, `vasti_r/vas_int_r-P3` (+5 more) | path_points/ConditionalPathPoint.py + utils.conditional_path_point_dependencies: moving point fitted to OpenSim's path (on while active, straight segment while not; Station 3 change 3.1); the on/off switch itself can't be reproduced, so it is approximated |
| approximated | ForceSet | Millard2012EquilibriumMuscle | `hamstrings_r`, `bifemsh_r`, `glut_max_r` (+15 more) | forces/Muscle.py: rigid-tendon MuJoCo muscle; F-L params fitted in Step 3; activation time constants kept; tendon compliance, fiber damping and F-V parameters not converted |
| approximated | ForceSet | MovingPathPoint | `rect_fem_r/rect_fem_r-P3`, `vasti_r/vas_int_r-P4`, `rect_fem_l/rect_fem_l-P3` (+1 more) | path_points/MovingPathPoint.py: child body with slide joints; each axis fitted with a quartic polynomial of its coordinate over the coordinate's range (path_points/utils.get_moving_path_point_dependency, Station 3 change 3.2); an axis moving < 1 mm is fixed mid-range (3.3) |
| approximated | JointSet | TransformAxis | `knee_r/translation1`, `knee_r/translation2`, `knee_l/translation1` (+1 more) | joints/CustomJoint.py: SimmSpline/NaturalCubicSpline -> quartic polynomial (fitted over the independent coordinate's range, Station 2 change 2.3) + joint equality |
| approximated | Model | gravity | `gravity` | OpenSim gravity (0 -9.8066499999999994 0) not converted; MuJoCo default (0 0 -9.81) used |
| converted | — | — | Body ×12, Mesh ×19, PathPoint ×48, Coordinate ×10, CustomJoint ×3, PinJoint ×5, TransformAxis ×14, WeldJoint ×4 | — |
