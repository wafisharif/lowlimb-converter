# Python API

```python
from osim2mujoco.convert import convert
from osim2mujoco.validate import validate

xml = convert("model.osim", "out/")                     # path to model_cvt3.xml
summary = validate("model.osim", xml, "out/validation")
```

```python
from osim2mujoco.support import build_report, to_markdown

report = build_report("model.osim", geometry="Geometry/")
print(report["outcome"], report["first_failure"])
print(to_markdown(report))
```

## Converting

```{eval-rst}
.. autofunction:: osim2mujoco.convert.convert

.. autoexception:: osim2mujoco.convert.ConversionError
```

## Validating

```{eval-rst}
.. autofunction:: osim2mujoco.validate.validate

.. autofunction:: osim2mujoco.validate.to_markdown
```

## Checking a model

```{eval-rst}
.. autofunction:: osim2mujoco.support.build_report

.. autofunction:: osim2mujoco.support.to_markdown
```

## Check modules

Each pair is split into an OpenSim side (writes a reference `.npz`) and a MuJoCo side (no OpenSim is
needed). Every module runs with `python -m osim2mujoco.checks.<module>`, and its docstring describes the method.

| Module | Compares |
| --- | --- |
| `osim_reference` / `mujoco_compare` | Body positions and orientations |
| `osim_paths` / `mujoco_paths` | Muscle lengths and moment arms |
| `osim_forces` / `mujoco_forces` | Static muscle forces |
| `osim_gait` / `mujoco_gait` | Lengths, moment arms, and joint moments along a walking trial |
| `osim_replay` / `mujoco_replay` | Open-loop replay of muscle excitations |
| `mass_properties` | Mass, center of mass, inertia |
| `limit_forces` | CoordinateLimitForce |
| `path_forces` | PathActuator and PathSpring |
| `contact_geometry` | Distances between contact shapes |
| `contact_dynamics` | Drops of the rigid model |
