# Station 0: Does MuJoCo support elastic tendons?

**Answer: no, not natively.** As of MuJoCo 3.15.0 (released 2026-10-05) and in the 2.3.7 baseline pin,
the built-in muscle has a rigid (inelastic) tendon. Elastic tendons are possible only through a
custom muscle model (user callbacks or an actuator plugin) that we would write ourselves.

## Evidence

**MuJoCo documentation** (`doc/modeling.rst`, commit `f1f216c`, 2026-10-05):

- "We assume that the biological tendon is inelastic, with constant length L_T, while the
  biological muscle length L_M varies over time."
- "We assume inelastic tendons while OpenSim can model tendon elasticity. We decided not to do that
  here, because tendon elasticity requires fast-equilibrium assumptions which in turn require various
  tweaks and are prone to simulation instability."
- The suggested approximation is to shorten the muscle operating range, which stretches the
  force-length curve (Zajac 1989). MyoConverter Step 3 already fits the operating range.
- Custom models: set `gaintype`/`biastype`/`dyntype` to `"user"` and provide callbacks;
  "Custom callbacks could then simulate elastic tendons or any other detail we have chosen to omit."

**Our test** (`test_elastic_tendon.py`, single muscle on a locked slider, full excitation):

| Check | MuJoCo 2.3.7 | MuJoCo 3.15.0 | Meaning |
| --- | --- | --- | --- |
| States per muscle | 1 | 1 | Activation only; no fiber/tendon length state |
| Force drift at fixed length, 1–2 s | 6.3e-9 | 6.4e-9 | Flat, so no tendon stretch dynamics |
| User bias callback from Python | works | works | Route to a custom elastic-tendon model exists |

## Consequence for the plan

- The plan's rule was "MuJoCo doesn't support elastic tendons → drop Station 6, note it as future work."
  Native support is absent, so **Station 6 is out of v1 scope by that rule.**
- A future custom model is feasible (Test 3), but would need its own validation, and Python
  callbacks are much slower than built-in muscles and do not run in MJX. Treat it as post-v1 research.
- A cheaper, in-scope relative: report how well the operating-range approximation reproduces
  OpenSim's elastic-tendon force-length curves, especially for ankle plantarflexors (long tendons).
  That fits inside Stations 4–5 with no new muscle model.
