import sys
from myoconverter.O2MPipeline import O2MPipeline

MODELS = {
    "gait10dof18musc": ("./models/osim/Gait10dof18musc/gait10dof18musc.osim",
                        "./models/osim/Gait10dof18musc/Geometry"),
    "gait2354": ("./models/osim/Gait2354Simbody/gait2354.osim",
                 "./models/osim/Gait2354Simbody/Geometry"),
}
name = sys.argv[1]
osim_file, geom = MODELS[name]
kwargs = dict(convert_steps=[1, 2, 3], muscle_list=None, osim_data_overwrite=True,
              conversion=True, validation=True, speedy=False, generate_pdf=True,
              add_ground_geom=True, treat_as_normal_path_point=False)
O2MPipeline(osim_file, geom, f"/app/data/{name}", **kwargs)
