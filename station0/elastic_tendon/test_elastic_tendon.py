"""Station 0: does MuJoCo's built-in muscle have an elastic tendon?

Test 1 (state count): a muscle with an elastic tendon needs a state for fiber (or tendon) length
  besides activation. If MuJoCo's muscle has exactly one state per muscle (activation),
  the tendon cannot be elastic.
Test 2 (isometric force): hold the actuator at a fixed length with constant excitation. With an
  elastic tendon the force rises over time as the fiber shortens and stretches the tendon.
  With a rigid tendon, force is a fixed function of activation and length, so once activation
  settles it stays flat.
Test 3 (route check): MuJoCo's documented route for custom muscle models is user callbacks
  (gaintype/biastype="user"). Check that the Python bindings can install one, so Station 6 could use it.

Usage: python test_elastic_tendon.py   (prints results; exits 1 if a check contradicts expectation)
"""
import sys
import numpy as np
import mujoco

XML = """
<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <site name="anchor" pos="0 0 0"/>
    <body name="load" pos="0.3 0 0">
      <joint name="slide" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.02" mass="1"/>
      <site name="tip" pos="0 0 0"/>
    </body>
  </worldbody>
  <tendon><spatial name="mtu"><site site="anchor"/><site site="tip"/></spatial></tendon>
  <actuator>
    <muscle name="m" tendon="mtu" force="1000" lengthrange="0.15 0.45"/>
  </actuator>
</mujoco>
"""

ok = True
m = mujoco.MjModel.from_xml_string(XML)
print(f"mujoco {mujoco.__version__}")

# Test 1
print(f"Test 1: actuators={m.nu}, actuator states (na)={m.na}  -> states per muscle = {m.na // m.nu}")
if m.na != m.nu:
    print("  UNEXPECTED: muscle has extra internal state"); ok = False
else:
    print("  activation is the only muscle state; no fiber/tendon length state exists")

# Test 2: lock the joint to make it isometric, excite fully, record force
m.jnt_range[0] = [0, 0]; m.jnt_limited[0] = 1
d = mujoco.MjData(m)
d.ctrl[0] = 1.0
f = []
for _ in range(2000):            # 2 s
    mujoco.mj_step(m, d)
    f.append(-d.actuator_force[0])
f = np.array(f)
late = f[1000:]                  # after activation has settled (tau_act ~ 10 ms)
drift = (late.max() - late.min()) / max(abs(late.mean()), 1e-9)
print(f"Test 2: isometric force over 1-2 s: mean {late.mean():.2f} N, relative drift {drift:.2e}, "
      f"actuator_length {d.actuator_length[0]:.4f} m (constant)")
if drift > 1e-6:
    print("  UNEXPECTED: force drifts at fixed length -> some internal length dynamics"); ok = False
else:
    print("  force is flat at fixed length: consistent with a rigid tendon")

# Test 3: install a user bias callback and confirm it changes the force
calls = {"n": 0}
def bias_cb(model, data, i):
    calls["n"] += 1
    return -123.0
m3 = mujoco.MjModel.from_xml_string(XML.replace(
    '<muscle name="m" tendon="mtu" force="1000" lengthrange="0.15 0.45"/>',
    '<general name="m" tendon="mtu" gaintype="fixed" gainprm="0" biastype="user" dyntype="none"/>'))
d3 = mujoco.MjData(m3)
mujoco.set_mjcb_act_bias(bias_cb)
try:
    mujoco.mj_forward(m3, d3)
    print(f"Test 3: user bias callback called {calls['n']}x, actuator_force = {d3.actuator_force[0]:.1f} N")
    if calls["n"] == 0 or abs(d3.actuator_force[0] + 123.0) > 1e-9:
        print("  UNEXPECTED: callback route did not work"); ok = False
    else:
        print("  callback route works from Python (custom tendon models are possible here)")
finally:
    mujoco.set_mjcb_act_bias(None)

print("ALL CHECKS AS EXPECTED" if ok else "SOME CHECKS CONTRADICT EXPECTATION")
sys.exit(0 if ok else 1)
