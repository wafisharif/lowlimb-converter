"""Run only the XML conversion step (Step 1) of the vendored pipeline, with the Station 1a settings.

Station 2 checks bodies and joints, which Step 1 sets and Steps 2-3 don't change (they only move muscle path
points and fit muscle parameters), so after a converter change only Step 1 needs to be rerun.
Usage: python station2/convert_xml_step.py <model.osim> <geometry_dir> <output_dir>
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from myoconverter.O2MPipeline import O2MPipeline  # noqa: E402

osim_path, geometry, out = (os.path.abspath(p) for p in sys.argv[1:4])
O2MPipeline(osim_path, geometry, out, convert_steps=[1], muscle_list=None, osim_data_overwrite=True,
            conversion=True, validation=False, speedy=False, generate_pdf=False,
            add_ground_geom=True, treat_as_normal_path_point=False)
