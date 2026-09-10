# GNCAirstrike PN Guidance + Monte Carlo Upgrade — Progress

## Working folder
`C:\Users\Meet\OneDrive\Desktop\Sem_5\Original_missile_guidance\GNCAirstrike`
(git repo, origin https://github.com/Vinayak-D/GNCAirstrike.git, branch `main`)
Mounted for device_bash at `$HOME/mnt/Sem_5/Original_missile_guidance/GNCAirstrike`.

Chosen over `Missile_guidance/simulink_model/GNCAirstrike-master` (a different,
messier copy with a half-done, wrong-headed PN attempt and two out-of-scope
files `guidance_controller.m`/`missile_gnc_sim.m` implementing forbidden
obstacle-avoidance path planning — those two files were DELETED per user
decision; that folder should not be touched further for this task).

Consolidation commit already made: "Consolidate project files from zip
extraction into repo root" (moved model.m/runfg.bat/simulink_model.slx/
simulink_model_FG.slx out of a nested zip-extraction folder into repo root).

## Checklist
- [x] 1. Read and understand existing model.m and .slx structure
- [x] 2. Implement PN guidance law (LOS angle, LOS rate, Vc, a_cmd) in model.m / Simulink
- [x] 3. Rewire PN output into existing Az reference, confirm terrain override still layers on top
- [x] 4. Regression check: eig(A-BK) and Kalman observer eigenvalues unchanged
- [x] 5. Single-trial end-to-end run: confirm convergence to target under PN
- [x] 6. Confirm terrain-override still triggers correctly near obstacle
- [ ] 7. Write monte_carlo.m with parsim-based batch execution
- [ ] 8. Run small batch (N=20) as a smoke test before full N=300 run
- [ ] 9. Run full Monte Carlo batch (N=300), save .mat/.csv results
- [ ] 10. Generate miss-distance histogram and trajectory overlay plots
- [ ] 11. Compute and write mean/std/success-rate/CEP50 into RESULTS.md

## Item 1 findings (model architecture — read this before touching anything)

MATLAB: R2025b. Toolboxes present: Control System, Aerospace (Toolbox+Blockset),
Mapping, Navigation, Robotics, Simscape(+Multibody), Simulink Control Design,
Statistics and ML, Symbolic Math, Simulink/MATLAB Coder.
**No Parallel Computing Toolbox** -> `parsim` will run serially, which is fine;
no parfor fallback needed, just use parsim/SimulationInput as instructed.

`simulink_model.slx` top level (unchanged block names — do not rename):
- `GUIDANCE COMMAND` — MATLAB Function (Stateflow EMChart), current signature
  `function [FPA,RNG,D_OBS,ALT,warn] = FPA(OBS, CURRENT, TARGET)`. 3 inputs:
  OBS <- `OBSTACLE LOCATION` Constant `[LAT_OBS LON_OBS]`;
  CURRENT <- `Mux1` = `[ALT; LAT; LON]` (muxes `POS > LLA` outputs, ALT first);
  TARGET <- `TARGET LOCATION` Constant `[LAT_TARGET LON_TARGET ELEV_TARGET]`.
  5 outputs: FPA -> `Add`(in2) + `FPA_CMD` Display + `Mux2` -> `FPA` Scope;
  RNG -> `RANGE-TARGET (m)` Display; D_OBS -> `DISTANCE-OBSTACLE (m)` Display;
  ALT -> `ALT < ELEV_TARGET (m)` (masked Compare-To-Constant -> `Stop` block:
  this is an IMPACT/ground-level sim-stop condition, NOT the 2km override —
  leave completely alone); warn -> `warning flag` Display only.
- **The terrain override is INLINE inside the FPA function itself**, not a
  separate Switch block: `thres=2000`; if `D_OBS < thres`: `warn=1`,
  `FPA = 0` (forces level flight) instead of `atan(dh/d)`. This *is* "the
  existing override" the task means by "ALT < ELEV_TARGET, DISTANCE-OBSTACLE,
  the 5-degree override" — the task calls it a 5° climb but the code as
  found forces 0 rad (level), not 5°. DECISION MADE: implement it as a
  literal 5° (5*pi/180 rad) climb per the task's explicit repeated wording,
  documented as a deliberate deviation from the as-found 0 rad. Trigger
  condition (D_OBS<2000m haversine) and the fact that it's an inline
  override on the guidance output are preserved exactly.
- `Add` = (`Missile LQG Controller...` outport #1 = `AoA`) + (`GUIDANCE
  COMMAND` outport #1 = FPA). `Sum` (config `+-`) = `Add` − (`Missile LQG
  Controller...` outport #6 = `theta_hat`). This synthesizes an FPA-tracking
  error ≈ `FPA_CMD − (theta_hat − AoA)` ≈ `FPA_CMD − gamma_hat` without a
  dedicated gamma estimator. Feeds `PID` (P=-1.0, I=0, D=0) -> `q_max/min`
  saturate -> `q_cmd` input of `Missile LQG Controller + Airframe Model`
  (inner LQR/Kalman loop — DO NOT TOUCH).
- **So the outer loop's reference is an ANGLE (radians, FPA), not literally
  "Az"** — the task's phrase "same Az reference units" is a loose/inaccurate
  description in the prompt. What must actually happen: convert PN's
  `a_cmd` (m/s^2) into an equivalent FPA angle command (radians) so `Add`
  needs no rewiring type-wise. Plan: `gamma_dot_cmd = a_cmd / Vm`, integrated
  over time into a persistent `gamma_cmd` state inside the chart (seeded
  from `atan(dh/d)` on the first call), output that as `FPA`.
- **Solver is `ode45`, Variable-step, StopTime=Inf, RelTol=1e-3.** dt is NOT
  fixed — do NOT hardcode dt=0.01 for the LOS-rate/closing-velocity finite
  differences. Must bring simulation time in via a new `Clock` block wired
  to a new chart input, and difference against a persistent previous time.

## Plan for item 2/3 (not yet implemented as of this checkpoint)
Rewrite the chart to (new signature):
`function [FPA,RNG,D_OBS,ALT,warn] = PN_GUIDANCE(OBS, CURRENT, TARGET, Vm, N_prime, t)`
- Add 3 new input ports: Vm <- reuse existing `VT` Constant (currently
  unwired, holds `Vel`); N_prime <- NEW Constant block wired to NEW model.m
  variable `N_prime = 3.5;` (comment: tunable, typical 3-4); t <- NEW `Clock`
  block.
- Keep existing d/dh/R(slant range)/lambda=atan2(dh,d)/D_OBS haversine math.
- persistent `R_prev, lambda_prev, t_prev, gamma_cmd` (init on first call:
  `t_prev=t; R_prev=R; lambda_prev=lambda; gamma_cmd=atan2(dh,d);` and just
  return that first-call value, no derivative yet, dt=0 guard).
- `dt = t - t_prev` (guard divide-by-zero); `lambda_dot = (lambda-lambda_prev)/dt`;
  `Vc = (R_prev - R)/dt` (negative range rate = closing velocity, positive
  while approaching); `a_cmd = N_prime*Vc*lambda_dot`; `gamma_dot_cmd =
  a_cmd/Vm`; `gamma_cmd = gamma_cmd + gamma_dot_cmd*dt`.
- If `D_OBS < 2000`: `warn=1`, output `FPA = 5*pi/180` and do **not** advance
  `gamma_cmd` that step (freeze the integrator so guidance resumes smoothly
  once clear of the obstacle) — else `warn=0`, output `FPA = gamma_cmd`,
  update persistents.
- `RNG = R` (unchanged meaning), `ALT = ELEV_CUR` (unchanged).
- model.m: add `N_prime = 3.5;` near the mission-geography section; nothing
  else in model.m changes (A,B,C,D,K,L,Qbar,Rbar all untouched — regression
  check in item 4 should show byte-identical eig() results).
- Wire: new line VT->GUIDANCE COMMAND port4, new Constant(N_prime)->port5,
  new Clock->port6. Use `add_line`/`add_block` in MATLAB, not manual GUI.

## Notes for resuming
- MATLAB is reached via the `mcp__remote-devices__MATLAB__*` tools, running
  live on the user's laptop; pass `project_path` =
  `C:\Users\Meet\OneDrive\Desktop\Sem_5\Original_missile_guidance\GNCAirstrike`.
- File edits to model.m / reading text files: use `device_bash` against
  `$HOME/mnt/Sem_5/Original_missile_guidance/GNCAirstrike/...` (same repo,
  different mount point — Linux VM on the user's machine, not this cloud
  container).
- Folder access + delete permission were already granted this session for
  `C:\Users\Meet\OneDrive\Desktop\Sem_5` — if a fresh session lacks these,
  request them again (device_request_folder_access /
  device_request_delete_permission) before working.
- Commit after each checklist item completes; this file is the resumability
  anchor — update it every time before running low on context.

## BLOCKING ISSUE found at item 5 (2026-09-09 session) -- READ BEFORE CONTINUING

Ran a single trial end-to-end (`sim(mdl,'StopTime','300','ReturnWorkspaceOutputs','on')`,
with signal logging added in-session via `set_param(portHandle,'DataLogging','on')`
on GUIDANCE COMMAND's 5 outputs + the x/z position integrators; Simulation Pace
S-Function block was commented out for the run since it has 0 ports/no signal
role and only paces wall-clock speed -- harmless to disable for batch runs,
NOT yet committed to the saved model, revisit before Monte Carlo).

Result: the missile does NOT converge under the new PN guidance -- RNG (slant
range) drops from ~79.4km to a minimum ~51.6km then climbs back out to ~260km
by t=300s; final altitude ends up ~268km (started at 10km, target at 0.795km
-- it climbed away instead of descending to the target 9.2km below).

**Diagnostic: reverted the chart script in-memory (not saved) to the exact
original static-FPA baseline and re-ran the identical test.** Result was
qualitatively the same divergence (min RNG ~36.7km then back out to ~267km,
final altitude ~268km). This proves the divergence is a PRE-EXISTING property
of the base project's kinematics/control wiring, not something the PN swap
introduced -- the PN implementation itself checks out (regression-clean,
correctly wired, override fires correctly at 5deg).

Suspected root cause (not yet fixed, NOT part of the task's scope as written
-- flagged to user, awaiting direction): the "angle" fed to the `sin`/`cos`
trig blocks that drive `dx/dt` and `dz/dt` is `theta - AoA` (via the `++`
block, `-+` config, standard gamma = theta - alpha identity, looks correct).
But block `VT*cos` (misleadingly named -- it actually computes `VT*sin(angle)`
because its input ports are `sin`, then `VT`) feeds `dx/dt>x`, and `VT*sin`
(misleadingly named -- actually computes `VT*cos(angle)`) feeds `dz/dt>z`.
With `theta` and `AoA` both IC=0 (so angle(0)=0): `x_dot(0) = VT*sin(0) = 0`,
`z_dot(0) = VT*cos(0) = VT` (full speed) -- i.e. at the very first instant
100% of velocity goes into the z/altitude channel and 0% into x/downrange,
which is backwards from the standard `x_dot=V*cos(gamma)`,
`z_dot=V*sin(gamma)` convention (small gamma near level flight should be
mostly downrange velocity, negligible vertical). This looks like a genuine
pre-existing sign/axis bug in the base repo (a public template,
github.com/Vinayak-D/GNCAirstrike), unrelated to guidance-law choice, and
was present before any of this session's edits.

Confirmed NOT the cause: LQR gain K, Kalman L, Qbar/Rbar, PID P=-1 gain,
Add/Sum wiring topology are all byte-identical to the original (see item 4
regression check) -- this is a kinematics-layer issue (the sin/cos/VT*sin/
VT*cos/dx/dt/dz/dt block cluster), not a controller-design issue, so fixing
it would NOT mean "changing the LQR/Kalman/PID design" in the sense the
task's non-goals forbid -- but it IS more than "replace the LOS/FPA command
block," so I stopped and asked the user rather than assuming license to fix
it. Their answer determines how items 5 onward proceed:
  (a) fix the apparent x/z trig-input swap (swap which trig-block output
      feeds which integrator) and re-verify both PN and a static-FPA sanity
      check converge, then continue the checklist, OR
  (b) leave the base physics exactly as found (even though it doesn't
      converge to intercept) and run the Monte Carlo / sanity checks as
      specified, reporting the true (non-converging) numbers honestly in
      RESULTS.md, OR
  (c) something else per user direction.

Current repo state on disk: `simulink_model.slx` HAS the working PN chart
(commit f66bc0e) but the Simulation Pace comment-out and any physics fix are
NOT YET saved/committed -- resume from commit f66bc0e, re-apply Simulation
Pace commenting (`set_param('simulink_model/Simulation Pace','commented','on')`)
each session since it's not yet persisted, and consult the user's answer to
the question above before proceeding.


## Update: kinematics fix applied, deeper control-loop issue found (same session)

Applied the approved fix: `dx/dt>x` now takes its input from the block
labeled `VT*sin` (which -- due to swapped block naming vs. actual wiring --
actually computes `VT*cos(angle)`), and `dz/dt>z` now takes its input from
a new `Gain_z_sign` block (gain -1) fed by the block labeled `VT*cos`
(which actually computes `VT*sin(angle)`). This makes `x_dot = VT*cos(gamma)`,
`z_dot = -VT*sin(gamma)` -- the sign on z accounts for `z` being a
NED-style "Down" state (`dz/dt` integrator IC = `-ELEV_INIT`, and `POS >
LLA` is an Aerospace Blockset FlatEarth2LLA block, which expects a
[N;E;D] input -- confirms the down-positive convention). **Not yet saved
to the .slx** (experiments were run in-memory) -- re-apply on resume:
```
dx_ph = get_param('simulink_model/dx//dt>x  (m)','PortHandles');
dz_ph = get_param('simulink_model/dz//dt>z  (m)','PortHandles');
delete_line(get_param(dx_ph.Inport(1),'Line'));
delete_line(get_param(dz_ph.Inport(1),'Line'));
vtcos_ph = get_param('simulink_model/VT*cos','PortHandles');
vtsin_ph = get_param('simulink_model/VT*sin','PortHandles');
add_line('simulink_model', vtsin_ph.Outport(1), dx_ph.Inport(1), 'autorouting','on');
add_block('simulink/Math Operations/Gain','simulink_model/Gain_z_sign','Position',[1560 645 1590 665]);
set_param('simulink_model/Gain_z_sign','Gain','-1');
gz_ph = get_param('simulink_model/Gain_z_sign','PortHandles');
add_line('simulink_model', vtcos_ph.Outport(1), gz_ph.Inport(1), 'autorouting','on');
add_line('simulink_model', gz_ph.Outport(1), dz_ph.Inport(1), 'autorouting','on');
```

Re-tested with the kinematics fix in place, first still with the ORIGINAL
static-FPA chart script (not PN) as a clean baseline sanity check:
- With outer PID `P=-1.0` (as found): still diverges, but differently --
  now flies BACKWARD (final x=-181054, min RNG occurs at t=0 and only
  gets worse) instead of climbing away.
- Experimentally tried `P=+1.0` (NOT saved/applied, pure diagnostic): loop
  becomes bounded/non-divergent but barely moves at all in 300s (x only
  reaches ~1660m, altitude only drops ~330m) -- essentially stalled, not
  tracking the small commanded FPA either.

**Traced why: `q_cmd` (the outer loop's output, INTO the
`Missile LQG Controller + Airframe Model` subsystem) is NOT a rate
command that some inner rate-loop tracks.** Inside that subsystem,
`Sum5 = q_cmd - (K_lqr * x_hat)`, and Sum5's output IS `delta_c` (elevator
deflection, feeds `B mult`/`B mult6` which are the `B*u`/`D*u` terms of
the state-space plant/observer). I.e. the outer loop's output is a direct
ADDITIVE BIAS on top of the standard LQR law `u=-Kx`, not a rate reference
run through its own tracking loop. Given `B(2)=-331.4` (large), even a
~0.1 rad bias (the typical magnitude of the geometry-derived FPA error)
produces a huge initial `q_dot`, saturating/violently exciting the fast
inner dynamics -- this is a **gain-tuning / architecture sensitivity
issue in the outer P loop itself**, independent of sign, independent of
guidance law (PN vs static FPA), and independent of the kinematics fix
above. Both `P=-1` and `P=+1` are unsatisfactory (diverge vs. stall).

**This is now squarely inside "control design"**, which the task's
non-goals explicitly protect ("Do not change the Kalman filter or LQR
gains/design", "leave navigation, control ... unchanged"). Stopped here
without changing the outer PID gain or any inner-loop element, and asked
the user for direction rather than guessing further. PID `P` was reset
back to `-1.0` (the as-found value) before stopping; nothing beyond the
kinematics fix above and the item-2/3 PN work (already committed in
f66bc0e) has been changed on disk.

Next step on resume: read the user's answer to the follow-up question
(asked after this checkpoint) about how to proceed with the outer-loop
tracking issue, then continue from there. Do not re-attempt random P-gain
guesses -- if authorized to touch the outer loop, do it analytically
(e.g. work out what `q_cmd` magnitude actually corresponds to a sane
elevator bias given B(2), and either add a small gain/scale on the FPA
error before it becomes `q_cmd`, or properly rate-limit/shape it) rather
than trial and error.


## RESOLVED: root cause of the blocking divergence (same session, continued)

My first kinematics "fix" (committed only in-memory, described in the two
sections above) was ITSELF WRONG -- diagnosed further and corrected:

**Real root cause**: the Simulink Trigonometry blocks literally named
`sin` and `cos` have their `Operator` parameter SWAPPED -- the block named
`sin` has `Operator=cos`, and the block named `cos` has `Operator=sin`
(confirmed via `get_param(blk,'Operator')`, not the block name). Because
of this, the `VT*cos`/`VT*sin` Product blocks downstream were ALREADY
computing exactly what their names say (`VT*cos(angle)` and
`VT*sin(angle)` respectively) -- my earlier port-order-based deduction
that they were mislabeled was wrong. So the ORIGINAL wiring
(`VT*cos`->`dx/dt`, `VT*sin`->`dz/dt`) was already `x_dot=VT*cos(gamma)`,
`z_dot=+VT*sin(gamma)` -- correct for x, but missing the negative sign on
z (needed because `dz/dt`'s integrator IC is `-ELEV_INIT` and `POS > LLA`
is an Aerospace Blockset FlatEarth2LLA block expecting NED [N;E;D] input,
i.e. `z` is really "Down" and altitude=`-z`, so `z_dot` must equal
`-h_dot = -V*sin(gamma)`).

**Undid the incorrect swap and applied the real, minimal fix**: `dx/dt`
restored to its original source (`VT*cos` block, unchanged); `dz/dt` now
fed through a single new `Gain_z_sign` block (gain = -1) inserted between
the original `VT*sin` block and the `dz/dt` integrator. That's the entire
kinematics change -- one sign flip on one line, nothing else.

**Also retuned the outer PID `P` gain magnitude** (approved by user,
"control tuning only, not LQR/Kalman"): analytically computed the DC gain
from a constant `q_cmd` bias to steady-state `q` via
`g = [-(A-B*K)^{-1}*B](2) = -1.126826` (using the untouched `A,B,K`), then
picked a target outer-loop time constant `tau=5s` (giving ~12x separation
from the -150 inner pole and ~2x from the -2.5 inner pole, well inside the
~77s nominal flight time) and solved `P = 1/(g*tau) = -0.1775` (numeric
value set on the `PID` block's `P` parameter: `-0.1774896923`). The SIGN
stayed negative, matching the original `P=-1.0` -- only the magnitude was
wrong (10x too aggressive, causing a genuine closed-loop instability given
the extreme plant sensitivity `B(2)=-331.4`, not a sign error).

Also commented out the `Simulation Pace` block (`commented='on'`) since
it's a standalone 0-port S-Function that only paces wall-clock speed for
live visualization -- irrelevant to results, but would make 300 batched
Monte Carlo runs take forever. This is a harmless, non-functional change.

**Verified with BOTH the original static-FPA chart and the PN chart**
(same corrected kinematics + retuned P in both cases):
- Original static FPA: converges cleanly, sim self-terminates via the
  `Stop` block at t=78.80s, final altitude=789.1m (target=795m), final
  slant range=1033.5m.
- PN guidance (the actual deliverable): converges cleanly, self-terminates
  at t=77.90s, final altitude=784.0m, final slant range=541.6m. Terrain
  override DID fire during this nominal run (`warn=1` at some point,
  min D_OBS=32.6m -- a close pass near the obstacle, confirms items 5 and
  6 simultaneously: PN converges to intercept AND the 2km/5-degree
  override engages correctly on a trajectory that passes near the
  obstacle, without needing a separately contrived test case).

Regression re-confirmed after all of the above: `eig(A-BK)` and the
Kalman observer eigenvalues are still byte-identical to the very first
baseline (`-2.5132,-150.1351` and `-18.2152,-15.9376`) -- none of this
touched `A,B,C,D,Q,R,K,Qbar,Rbar,L`.

Saved to `simulink_model.slx` and ready to commit. Next: build
`monte_carlo.m` (item 7) using `Simulink.SimulationInput` + `parsim` (no
Parallel Computing Toolbox on this machine, confirmed earlier -- parsim
will run trials serially, which satisfies the task's requirement without
needing the parfor fallback). Remember `Simulation Pace` is commented out
in the saved model already, so no per-trial override needed for that.
