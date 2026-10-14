"""Convert an OpenSim model to MuJoCo with the osim2mujoco pipeline.

A thin, safe wrapper around the vendored MyoConverter pipeline (src/myoconverter) with our fixes:

- the model is checked first (osim2mujoco.support), and a model the converter can't handle is rejected with
  the reason, before anything runs;
- the output folder must be new or empty, because the pipeline silently resumes Steps 2-3 from any _cvt2.xml /
  _cvt3.xml it finds there (station3/FINDINGS.md). The folder is claimed with a marker file created atomically, so
  two conversions can't share it; the marker is removed when the conversion ends;
- the converted model's path is returned.
"""
import os

from osim2mujoco import support

STEPS = (1, 2, 3)
MARKER = ".osim2mujoco-running"


class ConversionError(RuntimeError):
    """The model can't be converted, or the conversion didn't produce the expected file."""


def default_geometry(osim_path):
    """The `Geometry` folder next to the model file, if there is one."""
    g = os.path.join(os.path.dirname(os.path.abspath(osim_path)), "Geometry")
    return g if os.path.isdir(g) else None


def check_output_folder(out_dir):
    """Raise ConversionError unless out_dir is missing or empty (the pipeline would reuse old results)."""
    if os.path.exists(out_dir):
        if not os.path.isdir(out_dir):
            raise ConversionError(f"{out_dir} exists and is not a folder")
        content = os.listdir(out_dir)
        if content == [MARKER]:
            raise ConversionError(f"another conversion is using {out_dir} (or one was interrupted: delete "
                                  f"{os.path.join(out_dir, MARKER)})")
        if content:
            raise ConversionError(f"{out_dir} is not empty; use a new folder (the pipeline would silently resume "
                                  f"from old results found there)")


def claim_output_folder(out_dir):
    """Create out_dir if needed and claim it with a marker file (exclusive create); return the marker's path.

    Fails if another conversion holds the folder or if anything else is in it.
    """
    os.makedirs(out_dir, exist_ok=True)
    marker = os.path.join(out_dir, MARKER)
    try:
        with open(marker, "x"):
            pass
    except FileExistsError:
        raise ConversionError(f"another conversion is using {out_dir} (or one was interrupted: delete {marker})")
    if os.listdir(out_dir) != [MARKER]:
        os.remove(marker)
        raise ConversionError(f"{out_dir} is not empty; use a new folder (the pipeline would silently resume "
                              f"from old results found there)")
    return marker


def precheck(osim_path, geometry):
    """Support report for the model; raises ConversionError if the XML step is predicted to fail."""
    report = support.build_report(osim_path, geometry)
    if report["outcome"] != "converts":
        f = report["first_failure"]
        raise ConversionError(f"{report['model']} can't be converted: {f['kind']} {f['path']}: {f['reason']}")
    return report


def output_path(osim_path, out_dir, steps):
    """Path of the model file the pipeline writes last for these steps."""
    stem = os.path.splitext(os.path.basename(osim_path))[0]
    return os.path.join(os.path.abspath(out_dir), f"{stem}_cvt{max(steps)}.xml")


def convert(osim_path, out_dir, geometry=None, steps=STEPS, speedy=False, myoconverter_validation=False,
            pdf=False, add_ground_geom=True, muscle_list=None, check=True):
    """Convert ``osim_path`` into ``out_dir``; return the path of the converted MuJoCo model.

    :param steps: any of (1,), (1, 2), (1, 2, 3). 1 = XML conversion, 2 = muscle-path optimisation (MyoConverter's;
        a no-op for models without wrapping objects), 3 = muscle-force fit (MyoConverter's plus our refit).
    :param geometry: mesh folder (default: the ``Geometry`` folder next to the model).
    :param myoconverter_validation: also run MyoConverter's own validation plots (slow); ``pdf``: and its PDF report.
    :param add_ground_geom: add a ground plane (not added if the model has its own contact).
    :param check: run the support pre-check first (recommended).
    :raises ConversionError: the model can't be converted, or the output folder isn't new or empty.
    """
    steps = tuple(sorted(set(int(s) for s in steps)))
    if steps not in ((1,), (1, 2), (1, 2, 3)):
        raise ValueError("steps must be (1,), (1, 2) or (1, 2, 3)")
    if pdf and not myoconverter_validation:
        raise ValueError("pdf needs myoconverter_validation (the PDF is built from its plots)")
    osim_path = os.path.abspath(osim_path)
    if not os.path.isfile(osim_path):
        raise FileNotFoundError(f"{osim_path} not found")
    geometry = os.path.abspath(geometry) if geometry else default_geometry(osim_path)
    if geometry is None:
        raise ConversionError("no geometry folder given and no Geometry folder next to the model")
    check_output_folder(out_dir)                      # fail fast, before the pre-check
    if check:
        precheck(osim_path, geometry)
    marker = claim_output_folder(out_dir)

    try:
        from myoconverter.O2MPipeline import O2MPipeline   # heavy imports (OpenSim, MuJoCo) only when converting
        O2MPipeline(osim_path, geometry, os.path.abspath(out_dir), convert_steps=list(steps), muscle_list=muscle_list,
                    osim_data_overwrite=True, conversion=True, validation=myoconverter_validation, speedy=speedy,
                    generate_pdf=pdf, add_ground_geom=add_ground_geom, treat_as_normal_path_point=False)
    finally:
        os.remove(marker)
    out = output_path(osim_path, out_dir, steps)
    if not os.path.isfile(out):
        raise ConversionError(f"the pipeline finished but {out} was not written; see the log in {out_dir}")
    return out
