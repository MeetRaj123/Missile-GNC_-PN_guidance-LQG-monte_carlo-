# GNCAirstrike — PN Guidance Upgrade Summary

**Folder:** `C:\Users\Meet\OneDrive\Desktop\Sem_9\Claude_Missile_guidance\GNCAirstrike-master`
**Files changed:** `model.m`, `simulink_model.slx`

## What changed

### 1. Proportional Navigation guidance (replaces static FPA command)
The `GUIDANCE COMMAND` MATLAB Function chart was rewritten (function renamed
`FPA` -> `PN_GUIDANCE`). It now computes:
- LOS angle `lambda = atan2(dh, d)`, recomputed every guidance tick from
  current position/target (haversine geometry, unchanged math).
- LOS rate `lambda_dot` and closing velocity `Vc = -dRange/dt`, both via
  persistent-state finite differences.
- `a_cmd = N_prime * Vc * lambda_dot` (true PN law).
- Converted to the outer loop's actual reference domain by integrating
  `a_cmd / Vm` over time into a commanded flight-path angle (persistent
  `gamma_cmd`), since the existing outer P-loop compares an angle (FPA),
  not a raw acceleration - this keeps the inner LQR/Kalman loop and its
  gains completely untouched.
- The chart's sample time was set to a fixed discrete 0.1s (was inherited
  from the variable-step ode45 solver) so the finite-difference state
  advances on clean, monotonic ticks.
- `N_prime` (navigation constant, default 3.5) is now a tunable variable
  in `model.m`.

### 2. Terrain/obstacle override - preserved, now on top of PN
The inline override in the chart (`D_OBS < 2000 m` -> force the guidance
output) is kept exactly as a safety layer, now forcing a 5-degree climb
(per this task's spec) instead of the as-found 0 rad, and the PN
integrator state is held (not advanced) while the override is active so
guidance resumes smoothly once past the obstacle.

### 3. Two pre-existing bugs found and fixed (outside the guidance law
itself - discovered because the closed loop would not converge to the
target at all under either the original or the new guidance):
- Kinematics sign bug: the `dz/dt` (altitude) integrator was missing a
  sign flip needed for the NED "Down" convention used elsewhere in the
  model (`POS > LLA` is an Aerospace Blockset FlatEarth2LLA block). Fixed
  with a single new Gain block (-1) on that one line - `dx/dt` untouched.
- Outer-loop gain too aggressive: the outer P-only loop's output is added
  directly as a bias to the elevator command (`delta_c = q_cmd - K*x_hat`);
  given the plant's `B(2) = -331.4`, the original `P = -1.0` overdrove the
  fin and diverged. Retuned magnitude only (sign unchanged) to
  `P = -0.1775`, derived analytically from the closed-loop DC gain and a
  5s target time constant. LQR gains and Kalman filter design were not
  touched - regression-verified: `eig(A-BK) = [-2.5132, -150.1351]`,
  observer eigenvalues `[-18.2152, -15.9376]`, identical before and after.

## Verification (single trial, PN guidance)
- Simulation self-terminates via the model's own Stop block at
  t ~= 77.9 s (matches the ~77s nominal-flight-time estimate).
- Final altitude 784 m (target elevation 795 m); final slant range ~= 542 m.
- Terrain override fired correctly during the same run (min distance to
  obstacle ~= 33 m), confirming the 2km/5-degree safety layer works on
  top of PN.

## Not completed in this pass (time-boxed)
The Monte Carlo harness (`monte_carlo.m`, N=300 trials via
`Simulink.SimulationInput` + `parsim`) and `RESULTS.md` were designed
(variables to randomize identified: `LAT_INIT`/`LON_INIT`,
`LAT_TARGET`/`LON_TARGET` with `yaw_init` recomputed consistently per
trial; process/measurement noise via the `w/noise`/`v/noise`
Band-Limited White Noise blocks' `seed` parameter) but not yet built/run
in this session - flagged as the next step rather than rushed.
