"""Station 1a: run the vendored pipeline (src/myoconverter) on the modern stack.
Settings are identical to baseline/run_baseline.py; only paths differ.
Usage: python station1/run_port.py <model> <models_clone_dir> <output_dir> [geometry_dir]
  model: gait10dof18musc | gait2354 (models_clone_dir = MyoConverter clone at cadf380)
         Rajagopal2016 | RajagopalLaiUhlrich2023 (models_clone_dir = opensim-models clone at d9b05d4; geometry_dir
         required: the merged geometry folder, see station1/support/README.md)
"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from myoconverter.O2MPipeline import O2MPipeline

MODELS = {
    "gait10dof18musc": ("models/osim/Gait10dof18musc/gait10dof18musc.osim", "models/osim/Gait10dof18musc/Geometry"),
    "gait2354": ("models/osim/Gait2354Simbody/gait2354.osim", "models/osim/Gait2354Simbody/Geometry"),
    "Rajagopal2016": ("Models/Rajagopal/Rajagopal2016.osim", None),
    "RajagopalLaiUhlrich2023": ("Models/Rajagopal/RajagopalLaiUhlrich2023.osim", None),
}
name, upstream, out = sys.argv[1], sys.argv[2], os.path.abspath(sys.argv[3])
osim_rel, geom_rel = MODELS[name]
if len(sys.argv) > 4:
    geom_rel = os.path.abspath(sys.argv[4])
if geom_rel is None:
    sys.exit(f"{name}: pass the geometry folder (4th argument)")
# MyoConverter's own validation plots and PDF read the converted models and never write them; for the Rajagopal models
# they take hours, so they are off there (the gait models keep Station 1a's exact settings)
plots = name not in ("Rajagopal2016", "RajagopalLaiUhlrich2023")
kwargs = dict(convert_steps=[1, 2, 3], muscle_list=None, osim_data_overwrite=True,
              conversion=True, validation=plots, speedy=False, generate_pdf=plots,
              add_ground_geom=True, treat_as_normal_path_point=False)
os.chdir(upstream)  # upstream example paths are relative to the upstream repo root
O2MPipeline(osim_rel, geom_rel, out, **kwargs)
