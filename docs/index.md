# osim2mujoco

**Convert OpenSim musculoskeletal models to MuJoCo, and check how closely the result matches OpenSim.**

osim2mujoco takes an OpenSim model (`.osim`) and writes a MuJoCo model (`.xml`) with the same bones, joints, muscles
and contacts. Then it measures the difference: every body's position, every muscle's path and force, and, if you give
it a walking trial, the joint moments needed to walk. It's built for lower-limb gait models and is a fork of
[MyoConverter](https://github.com/MyoHub/myoconverter). It is an independent project, not affiliated with the OpenSim
or MuJoCo teams.

```bash
osim2mujoco check    model.osim                  # what will convert, and how (seconds)
osim2mujoco convert  model.osim out/ --validate  # convert, then compare with OpenSim (minutes)
```

## Why use it instead of MyoConverter

Every number below comes from a check that compares the converted model with OpenSim. The project's
{doc}`validation` page has the method and the full results.

| | osim2mujoco | MyoConverter |
| --- | --- | --- |
| OpenSim's own example model files that convert | **54 of 56** | 40 of 56 |
| Body masses and inertias | **exactly OpenSim's** | inflated for small bodies in 32 of its 40 conversions |
| Muscle moment arms, Rajagopal 2016 (mean error) | **0.46 mm** | 1.64 mm |
| Muscle force at full activation, Rajagopal 2016 (mean error, % of max force) | **2.5%** | 6.9% |
| Joint moments needed to walk, Rajagopal 2016 (mean error) | **0.0011 N·m/kg** | 0.0112 N·m/kg |
| Contact spheres and half-spaces, limit forces, path actuators and springs | **converted and checked** | skipped |
| Planar, free, ball and gimbal joints | **converted** | not supported |
| A report comparing the converted model with OpenSim | **every conversion: bodies, muscle paths, forces, other elements** | its own plots and PDF |

It also says before converting whether a model will convert, element by element, and refuses one it can't handle
with the reason, instead of failing halfway.

## What it can't do yet

Contact *dynamics* don't fully match OpenSim: the contact geometry is exact, and the contact stiffness, damping and
friction are fitted to OpenSim drops of the model, but on tilted or sliding drops the converted model parts from
OpenSim by several centimetres. MuJoCo's muscles have rigid tendons, and how muscle force changes with speed isn't
checked. The full list is on {doc}`limitations`.

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
