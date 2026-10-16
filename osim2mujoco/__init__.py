"""osim2mujoco: OpenSim -> MuJoCo conversion for lower-limb gait models, with validation reports.

Our own code, separate from the vendored MyoConverter in src/myoconverter:
  support   element-support report: what in an .osim file converts, and how (Station 1b)
  convert   safe wrapper around the conversion pipeline
  checks    comparisons of a converted model with OpenSim (Stations 2-4)
  validate  validation report built from those checks
  cli       the `o2m` command
"""
__version__ = "0.1.0.dev0"
