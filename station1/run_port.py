"""Station 1a: run the vendored pipeline (src/myoconverter) on the modern stack.
Settings are identical to baseline/run_baseline.py; only paths differ.
Usage: python station1/run_port.py <model> <upstream_clone_dir> <output_dir>
  model: gait10dof18musc | gait2354
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from myoconverter.O2MPipeline import O2MPipeline

MODELS = {
    "gait10dof18musc": ("models/osim/Gait10dof18musc/gait10dof18musc.osim", "models/osim/Gait10dof18musc/Geometry"),
    "gait2354": ("models/osim/Gait2354Simbody/gait2354.osim", "models/osim/Gait2354Simbody/Geometry"),
}
name, upstream, out = sys.argv[1], sys.argv[2], os.path.abspath(sys.argv[3])
osim_rel, geom_rel = MODELS[name]
kwargs = dict(convert_steps=[1, 2, 3], muscle_list=None, osim_data_overwrite=True,
              conversion=True, validation=True, speedy=False, generate_pdf=True,
              add_ground_geom=True, treat_as_normal_path_point=False)
os.chdir(upstream)  # upstream example paths are relative to the upstream repo root
O2MPipeline(osim_rel, geom_rel, out, **kwargs)
