"""o2m (osim2mujoco): convert OpenSim gait models to MuJoCo and report how closely the result matches.

  o2m check    MODEL.osim [--geometry DIR] [--md FILE] [--json FILE]
  o2m convert  MODEL.osim OUT_DIR [--geometry DIR] [--steps 1 2 3] [--validate] [...]
  o2m validate MODEL.osim CONVERTED.xml OUT_DIR [--levels kinematics paths forces elements]
                           [--ik TRIAL_IK.mot --grf TRIAL_GRF.xml [--start S --end S] [--controls CONTROLS.sto]]

Exit codes: 0 success; 1 the model can't be converted / a check failed; 2 bad input.
"""
import argparse
import os
import sys

from osim2mujoco import __version__


def _quiet_logging(verbose):
    """Unless --verbose: warnings and errors only from the pipeline (loguru) and OpenSim; the pipeline still writes its
    full log to <out_dir>/<model>_conversion.log."""
    if verbose:
        return
    import warnings
    warnings.filterwarnings("ignore", message="The default value of `algorithm`")   # PyVista, in mesh conversion
    try:
        from loguru import logger
        logger.remove()
        logger.add(sys.stderr, level="WARNING")
    except ImportError:
        pass
    try:
        import opensim
        opensim.Logger.setLevelString("warn")
    except (ImportError, AttributeError):
        pass


def _cmd_check(a):
    from osim2mujoco import support
    argv = [a.osim] + (["--geometry", a.geometry] if a.geometry else []) + \
        (["--md", a.md] if a.md else []) + (["--json", a.json] if a.json else [])
    return support.main(argv)


def _cmd_convert(a):
    from osim2mujoco import convert as cv
    try:
        walking = _walking_args(a)            # checked before converting, so a bad trial fails fast
        for k in ("ik", "grf", "controls"):
            if walking and walking.get(k) and not os.path.isfile(walking[k]):
                raise FileNotFoundError(walking[k])
        if walking and not a.validate:
            raise ValueError("--ik/--grf are for the validation report: add --validate")
        out = cv.convert(a.osim, a.out_dir, geometry=a.geometry, steps=a.steps, speedy=a.speedy,
                         myoconverter_validation=a.myoconverter_report, pdf=a.myoconverter_report,
                         add_ground_geom=not a.no_ground)
    except cv.ConversionError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except (ValueError, FileNotFoundError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    print(f"converted model: {out}")
    if not a.validate:
        return 0
    from osim2mujoco import validate as va
    levels = ("kinematics", "elements") if sorted(set(a.steps)) == [1] else va.LEVELS   # no muscles fitted yet
    if walking and "paths" in levels:
        levels = tuple(levels) + ("walking",)
    report_dir = os.path.join(a.out_dir, "validation")
    r = va.validate(a.osim, out, report_dir, levels=levels, walking=walking)
    print(f"validation report: {os.path.join(report_dir, 'report.md')}")
    return 0 if r["ok"] else 1


def _walking_args(a):
    """The walking trial from --ik/--grf (both needed), or None."""
    if not a.ik and not a.grf:
        return None
    if not (a.ik and a.grf):
        raise ValueError("a walking trial needs both --ik and --grf")
    return {"ik": a.ik, "grf": a.grf, "start": a.start, "end": a.end, "controls": a.controls}


def _cmd_validate(a):
    from osim2mujoco import validate as va
    try:
        walking = _walking_args(a)
        levels = list(a.levels)
        if walking and "walking" not in levels:
            levels.append("walking")
        r = va.validate(a.osim, a.xml, a.out_dir, levels=levels, walking=walking)
    except (FileNotFoundError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    print(f"validation report: {os.path.join(a.out_dir, 'report.md')}")
    return 0 if r["ok"] else 1


def _add_walking_args(p):
    g = p.add_argument_group("walking trial (adds the walking level to the validation report)")
    g.add_argument("--ik", help="OpenSim inverse-kinematics result of the trial (.mot or .sto)")
    g.add_argument("--grf", help="its ground reaction forces as an OpenSim ExternalLoads file (.xml)")
    g.add_argument("--start", type=float, help="start of the part to compare (s; default: the whole trial)")
    g.add_argument("--end", type=float, help="end of the part to compare (s)")
    g.add_argument("--controls", help="muscle excitations (.sto, e.g. OpenSim CMC's *_controls.sto): also replay "
                                      "them open loop in both simulators (reported only)")


def build_parser():
    ap = argparse.ArgumentParser(prog="o2m", description=__doc__.split("\n\n")[0])
    ap.add_argument("--version", action="version", version=f"osim2mujoco {__version__}")
    ap.add_argument("-v", "--verbose", action="store_true", help="show the pipeline's and OpenSim's full log")
    sub = ap.add_subparsers(dest="command", required=True)

    c = sub.add_parser("check", help="list every element of an OpenSim model and whether it converts (fast, "
                                     "no conversion)")
    c.add_argument("osim")
    c.add_argument("--geometry", help="mesh folder; checks every mesh file exists")
    c.add_argument("--md", help="write the report as Markdown")
    c.add_argument("--json", help="write the report as JSON")
    c.set_defaults(func=_cmd_check)

    v = sub.add_parser("convert", help="convert an OpenSim model to MuJoCo")
    v.add_argument("osim")
    v.add_argument("out_dir", help="new or empty folder for the converted model")
    v.add_argument("--geometry", help="mesh folder (default: the Geometry folder next to the model)")
    v.add_argument("--steps", type=int, nargs="+", default=[1, 2, 3], choices=[1, 2, 3],
                   help="1 = XML conversion only (seconds), 1 2 3 = also fit muscle paths and forces (minutes; "
                        "default)")
    v.add_argument("--validate", action="store_true",
                   help="then write a validation report into OUT_DIR/validation (adds minutes)")
    v.add_argument("--speedy", action="store_true", help="MyoConverter's faster, less thorough optimisation")
    v.add_argument("--myoconverter-report", action="store_true",
                   help="also run MyoConverter's own validation plots and PDF (slow)")
    v.add_argument("--no-ground", action="store_true", help="don't add a ground plane geom")
    _add_walking_args(v)
    v.set_defaults(func=_cmd_convert)

    r = sub.add_parser("validate", help="compare a converted MuJoCo model with its OpenSim original")
    r.add_argument("osim")
    r.add_argument("xml", help="the converted MuJoCo model (the _cvt3.xml for paths and forces)")
    r.add_argument("out_dir", help="folder for report.md, report.json and the detailed results")
    r.add_argument("--levels", nargs="+", default=["kinematics", "paths", "forces", "elements"],
                   choices=["kinematics", "paths", "forces", "elements", "walking"])
    _add_walking_args(r)
    r.set_defaults(func=_cmd_validate)
    return ap


def main(argv=None):
    a = build_parser().parse_args(argv)
    if a.command != "check":
        _quiet_logging(a.verbose)
    return a.func(a)


if __name__ == "__main__":
    sys.exit(main())
