"""Comparisons of a converted MuJoCo model with its OpenSim original (Stations 2-4 of the project pipeline).

Each check has an OpenSim side, which needs OpenSim and writes a reference .npz, and a MuJoCo side, which needs only
MuJoCo and NumPy (so it also runs in MuJoCo 2.3.7 on MyoConverter's own output) and compares against the reference:

  osim_reference / mujoco_compare   body kinematics (Station 2)
  osim_paths     / mujoco_paths     muscle lengths and moment arms (Station 3)
  osim_forces    / mujoco_forces    static muscle forces (Station 4)

Methods and gates: station2/GATES.md, station3/GATES.md, station4/GATES.md.
"""
