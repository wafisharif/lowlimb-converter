# osim2mujoco

osim2mujoco converts OpenSim models (`.osim`) to MuJoCo (`.xml`) models, and checks the results directly against OpenSim; this includes body
positions, muscle paths, muscle forces and, given a walking trial, the joint moments needed to walk. osim2mujoco is built for
lower-limb gait models and originally began as an extension of [MyoConverter](https://github.com/MyoHub/myoconverter). It isn't
affiliated with the OpenSim or MuJoCo teams.

```bash
osim2mujoco check    model.osim                  # what will convert (seconds)
osim2mujoco convert  model.osim out/ --validate  # convert, then compare with OpenSim (minutes)
```

## Compared with MyoConverter

| | osim2mujoco | MyoConverter |
| --- | --- | --- |
| OpenSim example models that convert | 54 of 56 | 40 of 56 |
| Body masses and inertias | exact | inflated for small bodies in 32 of 40 |
| Moment-arm error, Rajagopal 2016 | 0.46 mm | 1.64 mm |
| Force error at full activation, Rajagopal 2016 | 2.5% of max force | 6.9% |
| Joint-moment error while walking, Rajagopal 2016 | 0.0011 N·m/kg | 0.0112 N·m/kg |
| Contact, limit forces, path actuators and springs | converted | skipped |
| Planar, free, ball and gimbal joints | converted | not supported |
| Validation report | every conversion | plots and a PDF from its own fitting steps |

Before converting, it also tells you if a model will convert and, if it won't, stops converting and lists the reason. Method and
full results: {doc}`validation`.

## Known gaps

Contact geometry is exact, but contact dynamics are not. On tilted or sliding drops, the converted model ends up several
centimetres from OpenSim's. MuJoCo muscles have rigid tendons, and force–velocity behaviour isn't checked. See
{doc}`limitations`.

```{toctree}
:maxdepth: 2
:caption: Using osim2mujoco

installation
quickstart
commands
report
supported
python-api
```

```{toctree}
:maxdepth: 2
:caption: How it's checked

validation
differences
limitations
```

```{toctree}
:maxdepth: 2
:caption: Project

developers
citing
```
