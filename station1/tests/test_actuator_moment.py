"""_actuator_moment_dense() on the modern stack must reproduce MuJoCo 2.3.7's dense actuator_moment.
Reference: actuator_moment_ref_mujoco237.json, written by MuJoCo 2.3.7 for the model and poses below
(3 actuators: a 2-joint muscle path, a motor, a muscle; 3 hinge joints; 3 poses; 19 nonzero entries).
Run: python station1/tests/test_actuator_moment.py
"""
import json, os, sys
import numpy as np
import mujoco
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "src"))
from myoconverter.optimization.utils.UtilsMujoco import _actuator_moment_dense

XML = '''<mujoco><worldbody><site name="a" pos="0 0 .3"/><body><joint name="j1" axis="0 1 0"/><joint name="j2" axis="1 0 0"/><geom size=".1"/><site name="b" pos=".1 0 .1"/>
<body pos="0 0 -.2"><joint name="j3" axis="0 0 1"/><geom size=".05"/><site name="c" pos=".05 .05 0"/></body></body></worldbody>
<tendon><spatial name="t"><site site="a"/><site site="b"/><site site="c"/></spatial><spatial name="t2"><site site="a"/><site site="c"/></spatial></tendon>
<actuator><muscle name="mu" tendon="t" lengthrange="0.05 0.9"/><motor name="mo" joint="j2"/><muscle name="mu2" tendon="t2" lengthrange="0.05 0.9"/></actuator></mujoco>'''
POSES = ([.3, .2, -.4], [-.7, .5, 1.1], [0, 0, 0])

ref = np.array(json.load(open(os.path.join(HERE, "actuator_moment_ref_mujoco237.json"))))
m = mujoco.MjModel.from_xml_string(XML); d = mujoco.MjData(m)
got = []
for q in POSES:
    d.qpos[:] = q; mujoco.mj_step(m, d)
    got.append(_actuator_moment_dense(m, d).copy())
err = np.abs(np.array(got) - ref).max()
print(f"mujoco {mujoco.__version__}: max |dense - 2.3.7 reference| = {err:.3g}")
assert ref.shape == (3, 3, 3) and (np.abs(ref) > 0).sum() == 19
assert err < 1e-12, "actuator_moment rebuild differs from MuJoCo 2.3.7"
print("PASS")
