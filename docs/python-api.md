# Python API

Everything the commands do is available from Python.

```python
from osim2mujoco.convert import convert
from osim2mujoco.validate import validate

xml = convert("model.osim", "out/")                     # returns the path of model_cvt3.xml
summary = validate("model.osim", xml, "out/validation")  # writes report.md / report.json, returns the summary
```

```python
from osim2mujoco.support import build_report, to_markdown

report = build_report("model.osim", geometry="Geometry/")
print(report["outcome"], report["first_failure"])        # "converts" or "fails", and where
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

## Checking before converting

```{eval-rst}
.. autofunction:: osim2mujoco.support.build_report

.. autofunction:: osim2mujoco.support.to_markdown
```

## The checks

Each check has an OpenSim side, which needs OpenSim and writes a reference `.npz`, and a MuJoCo side, which needs only
MuJoCo and NumPy and compares against it. Each module also runs on its own (`python -m osim2mujoco.checks.<module>`);
its docstring describes the method.

| Module | Station | What |
| --- | --- | --- |
| `osim_reference` / `mujoco_compare` | 2 | Body positions and orientations in the same poses |
| `osim_paths` / `mujoco_paths` | 3 | Muscle lengths and moment arms |
| `osim_forces` / `mujoco_forces` | 4 | Static muscle forces at four activation levels |
| `osim_gait` / `mujoco_gait` | 5 | Lengths, moment arms and joint moments along a walking trial |
| `osim_replay` / `mujoco_replay` | 5 | Open-loop replay of recorded muscle excitations |
| `mass_properties` | 6 | Mass, centre of mass and inertia of every body |
| `limit_forces` | 6 | CoordinateLimitForce over a sweep |
| `path_forces` | 6 | PathActuator and PathSpring lengths and forces |
| `contact_geometry` | 6 | Signed distance of every contact pair |
| `contact_dynamics` | 6 | Rigid drops of the model in both simulators |
