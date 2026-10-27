# Rectification and verification report

## Environment

- Branch: `test`, tracking `origin/test`.
- MATLAB/Simulink: R2026a executable; models loaded successfully.
- MATLAB path used for dataset generation, validation, ANFIS training, ODE comparison, and Simulink smoke/full tests.

## Gate results

| Check | Result | Evidence |
|---|---|---|
| Same-plant dataset generation | PASS | 24 scenarios, 480,000 raw samples, 48,000 subsampled rows |
| Dataset validation | PASS | schema, finite values, duty bounds, run independence, plausibility |
| ANFIS training | PASS | 32,000 train / 8,000 validation / 8,000 held-out test rows |
| Early stopping | PASS | validation minimum at epoch 9; validation RMSE 0.10254 |
| Held-out test | PASS | RMSE 0.07111, MAE 0.05836, max error 0.32934 |
| Individual Simulink smoke tests | PASS | open-loop, PID, Fuzzy, ANFIS all simulated 5 ms without errors |
| Unified Simulink full run | PARTIAL | finite 300 ms logs; PID/Fuzzy regulate, ANFIS remains inaccurate |
| Held-out disturbance script | PASS | four line-change profiles completed for all controllers |

## Unified Simulink result

The corrected unified model uses duty limits `[0.05, 0.95]` and derivative scale `1/200` for both Fuzzy and ANFIS. At the nominal 300 ms run:

| Controller | Final Vout (V) | Peak Vout (V) | Finite log |
|---|---:|---:|---|
| PID | 40.0097 | 66.4366 | yes |
| Fuzzy | 39.9703 | 66.2969 | yes |
| ANFIS | 49.6332 | 89.7246 | yes |

The ANFIS result is a real remaining defect in the branch's Simulink deployment/training interaction. The previous input-range warning was fixed: the trained FIS now declares both inputs as `[-1,1]`, and the full run completes without the former out-of-range warning flood.

## MATLAB averaged-backend comparison

For the 20 V to 25 V line-step case, the generated `Data/Results/comparison_results.csv` reports: PID rise 2.47 ms, overshoot 51.28%, SSE 4.956 V, IAE 2.347; Fuzzy rise 2.47 ms, overshoot 58.26%, SSE 6.776 V, IAE 3.434; ANFIS rise 2.34 ms, overshoot 52.16%, SSE 2.397 V, IAE 2.238. These values are useful for debugging but are not substituted for the Simulink result above.
