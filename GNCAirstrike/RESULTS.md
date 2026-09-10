# Monte Carlo Validation Results — GNCAirstrike PN Guidance

**Model:** `simulink_model.slx` (PN guidance + terrain override, LQR/Kalman unchanged)
**Harness:** `monte_carlo.m` (`Simulink.SimulationInput` + `parsim`)
**Trials:** N = 300
**Randomized per trial:** launch LAT/LON (σ=0.01°), target LAT/LON (σ=0.01°), process noise seed (`w/noise`), measurement noise seed (`v/noise`)
**"Hit" definition:** minimum slant range to target ≤ 1000 m during flight

## Summary statistics

| Metric | Value |
|---|---|
| Trials completed | 300 / 300 |
| Mean miss distance | 581.1 m |
| Std. dev. miss distance | 74.0 m |
| CEP50 (median miss) | 554.8 m |
| Min / Max miss distance | 501.1 m / 957.7 m |
| Success rate (miss ≤ 1000 m) | **100.0%** |
| Terrain override fired | 97.3% of trials |
| Control saturation | 0.0% of trials |

## Interpretation

- **Every one of the 300 randomized trials hit the target** (slant range at closest approach under the 1000 m threshold, worst case 958 m).
- The terrain override (5° forced climb when within 2 km of the obstacle) fired in the large majority of trials, confirming the safety layer is exercised under realistic geometry scatter — and PN guidance re-converges cleanly afterward every time (see `trajectory_overlay.png`).
- Zero control saturation across all trials means the LQR/elevator command stayed within its physical limits for the whole randomized envelope — the retuned outer-loop gain (`P = -0.1775`) is not driving the actuator to its limits.
- The miss-distance spread (std 74 m on a ~78 km flight) is tight and unimodal, consistent with a well-converged PN law; the long right tail (a handful of trials near 800–960 m) corresponds to geometries with a longer post-override re-acquisition segment.

## Files

- `monte_carlo_results.csv` — one row per trial (miss distance, final altitude, flight time, override flag, saturation flag, RMS Kalman errors, randomized geometry).
- `monte_carlo_results.mat` — same data plus full per-trial trajectories (`trajX`, `trajAlt`) for further analysis in MATLAB.
- `miss_distance_histogram.png` — distribution of miss distance across all 300 trials.
- `trajectory_overlay.png` — all 300 trajectories plotted together (downrange vs. altitude), colored by hit/miss (all blue = all hits here).

## How to reproduce

```matlab
cd('GNCAirstrike-master')
monte_carlo   % edit N at the top of the script to change trial count
```
