"""Dump the physics-relevant arrays of a compiled MuJoCo model to .npz, for cross-version comparison.
Usage: python dump_compiled.py <model.xml> <out.npz> [geometry_dir_override]
Works with MuJoCo 2.3.7 and 3.x. Arrays that exist in only one version are skipped by the comparer."""
import sys, os, re, numpy as np, mujoco
xml, out = sys.argv[1], sys.argv[2]
s = open(xml).read()
if len(sys.argv) > 3:
    s = s.replace('file="Geometry/', f'file="{os.path.abspath(sys.argv[3])}/')
os.chdir(os.path.dirname(os.path.abspath(xml)))
m = mujoco.MjModel.from_xml_string(s)
names = {}
FIELDS = """nq nv nu nbody njnt ngeom nsite ntendon nwrap neq nmesh nkey
body_parentid body_jntnum body_pos body_quat body_ipos body_iquat body_mass body_inertia
jnt_type jnt_bodyid jnt_pos jnt_axis jnt_range jnt_limited jnt_stiffness qpos0 dof_damping dof_armature
geom_type geom_bodyid geom_size geom_pos geom_quat geom_group geom_contype geom_conaffinity
site_bodyid site_pos site_size
tendon_adr tendon_num tendon_limited tendon_range tendon_width tendon_stiffness tendon_damping tendon_lengthspring
wrap_type wrap_objid wrap_prm
eq_type eq_obj1id eq_obj2id eq_data eq_solimp eq_solref
actuator_trntype actuator_dyntype actuator_gaintype actuator_biastype actuator_trnid actuator_dynprm
actuator_gainprm actuator_biasprm actuator_ctrlrange actuator_lengthrange actuator_forcerange
key_qpos npair pair_geom1 pair_geom2""".split()
arr = {}
for f in FIELDS:
    if f == "eq_active": continue
    if hasattr(m, f):
        arr[f] = np.array(getattr(m, f))
arr["eq_active_init"] = np.array(m.eq_active0 if hasattr(m, "eq_active0") else m.eq_active)
arr["opt_timestep"] = np.array(m.opt.timestep); arr["opt_gravity"] = np.array(m.opt.gravity)
for kind, n in [(mujoco.mjtObj.mjOBJ_BODY, m.nbody), (mujoco.mjtObj.mjOBJ_JOINT, m.njnt), (mujoco.mjtObj.mjOBJ_GEOM, m.ngeom),
                (mujoco.mjtObj.mjOBJ_SITE, m.nsite), (mujoco.mjtObj.mjOBJ_TENDON, m.ntendon), (mujoco.mjtObj.mjOBJ_ACTUATOR, m.nu)]:
    arr["names_" + str(kind).split(".")[-1]] = np.array([mujoco.mj_id2name(m, kind, i) or "" for i in range(n)])
np.savez(out, **arr); print(f"mujoco {mujoco.__version__}: dumped {len(arr)} arrays")
