"""Station 5 for Rajagopal 2016: build its walking trial from OpenSim's own Rajagopal pipeline data.

opensim-models @ d9b05d4 ships the raw trial (`Pipelines/Rajagopal/ExpData`: marker trajectories and ground reaction
forces of one subject walking) and the pipeline's setup files, but not the inverse-kinematics result. This rebuilds it
with OpenSim's own tools and the pipeline's own settings:
1. the generic Rajagopal2016 model scaled with the pipeline's applied scale factors (`Scale/scaleSet_applied_walk.xml`)
   and given the pipeline's subject marker set (`Scale/subject_scaled_walk_markerset.xml`);
2. OpenSim's InverseKinematicsTool with the pipeline's IK setup (`IK/ik_setup_walk.xml`: its marker weights and time
   range) on that model and `ExpData/motion_capture_walk.trc`;
3. the ground reaction forces as the pipeline's ID setup has them (`ID/grf_walk.xml` with `ExpData/grf_walk.mot`),
   copied next to the IK result with the data file path made local.
Station 5 then replays these coordinates on the generic model over the ID setup's time range (0.45-1.8 s), as it
does for the gait models (data recorded for a scaled subject, replayed on the generic model: both simulators get
exactly the same inputs). Deterministic: the same files give the same result.

Usage: python station5/make_rajagopal_trial.py <opensim_models_clone_dir> <out_dir>
Writes <out_dir>/rajagopal_walk_ik.mot, rajagopal_walk_grf.xml, grf_walk.mot and the scaled model it used.
"""
import os
import shutil
import sys

import opensim as osim
from lxml import etree


def main():
    osm, out = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
    os.makedirs(out, exist_ok=True)
    pipe = os.path.join(osm, "Pipelines", "Rajagopal")
    model = osim.Model(os.path.join(osm, "Models", "Rajagopal", "Rajagopal2016.osim"))
    state = model.initSystem()
    scale_set = osim.ScaleSet(os.path.join(pipe, "Scale", "scaleSet_applied_walk.xml"))
    model.scale(state, scale_set, False, -1.0)
    markers = osim.MarkerSet(os.path.join(pipe, "Scale", "subject_scaled_walk_markerset.xml"))
    model.updateMarkerSet(markers)
    model.finalizeConnections()
    scaled = os.path.join(out, "rajagopal_walk_scaled.osim")
    model.printToXML(scaled)

    ik = osim.InverseKinematicsTool(os.path.join(pipe, "IK", "ik_setup_walk.xml"))
    ik_model = osim.Model(scaled)       # keep a reference: the tool holds a raw pointer to it
    ik.setModel(ik_model)
    ik.setMarkerDataFileName(os.path.join(pipe, "ExpData", "motion_capture_walk.trc"))
    ik_out = os.path.join(out, "rajagopal_walk_ik.mot")
    ik.setOutputMotionFileName(ik_out)
    ik.set_report_errors(False)
    ik.setResultsDir(out)
    ik.run()

    shutil.copyfile(os.path.join(pipe, "ExpData", "grf_walk.mot"), os.path.join(out, "grf_walk.mot"))
    t = etree.parse(os.path.join(pipe, "ID", "grf_walk.xml"))
    loads = t.getroot().find("ExternalLoads")
    loads.find("datafile").text = "grf_walk.mot"
    kin = loads.find("external_loads_model_kinematics_file")
    if kin is not None:
        loads.remove(kin)
    t.write(os.path.join(out, "rajagopal_walk_grf.xml"))
    print(f"IK: {ik_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
