# ANFIS Dataset Generation & Validation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a fully reproducible, verified, physics-compliant supervised dataset generation, validation, and training pipeline for the ANFIS-controlled DC–DC boost converter.

**Architecture:** 
1. Use the verified averaged continuous-conduction-mode (CCM) boost converter equations ($L \cdot \frac{di_L}{dt} = V_{in} - (1-D)V_{out} - r_L i_L$, $C \cdot \frac{dV_{out}}{dt} = (1-D)i_L - \frac{V_{out}}{R}$) with $V_{in}=20\text{V}, V_{ref}=40\text{V}, L=500\ \mu\text{H}, C=1000\ \mu\text{F}, R=500\ \Omega, r_L=0.1\ \Omega$.
2. Implement an anti-windup, saturation-clamped reference controller ($D \in [0.05, 0.95]$) to track setpoints under diverse operating conditions (startup, setpoint tracking, load variations, line voltage disturbances).
3. Log full trajectories with `run_id`, split cleanly across independent runs (70% train, 15% validation, 15% test), perform strict automated numerical and physical validation, and train ANFIS using hybrid learning.

**Tech Stack:** MATLAB R2026a, Simulink, Fuzzy Logic Toolbox (`genfis`, `anfis`, `evalfis`), Control System Toolbox, Simscape Electrical.

**Spec:** User Master Prompt "GENERATE A VALIDATED ANFIS TRAINING DATASET FOR THE BOOST CONVERTER PROJECT"

## Global Constraints
- Target reference voltage: $V_{ref} = 40\text{ V}$ nominal (with sweeps e.g. $35\text{V} - 45\text{V}$).
- Input voltage: $V_{in} = 20\text{ V}$ nominal (with sweeps $15\text{V} - 25\text{V}$).
- Inductor & parasitic: $L = 500\ \mu\text{H}, r_L = 0.1\ \Omega$.
- Capacitor & load: $C = 1000\ \mu\text{F}, R = 500\ \Omega$ (with load sweeps $250\ \Omega - 750\ \Omega$).
- Steady-state operating point: $D \approx 0.502, i_L \approx 0.16\text{ A}, V_{out} \approx 40\text{ V}$.
- Trajectory splitting: splits must be partitioned by complete simulation runs (`run_id`), never by random shuffling of consecutive time-steps.
- Inputs to ANFIS: $e = V_{ref} - V_{out}$ and $\Delta e = \frac{e(k) - e(k-1)}{\Delta t}$ (derivative of error per second, or discrete difference per sample, clearly documented).
- Output/Target: Duty cycle command $D \in [0.05, 0.95]$.

## Review Focus
1. Non-causal or leaking data splits: ensure zero temporal overlap across train/val/test splits by using whole-run grouping.
2. Inconsistent plant parameters: ensure `boost_converter_params.m`, `boost_converter_ode.m`, and Simulink `boost_converter_openloop.slx` share identical physics and parasitic resistance $r_L = 0.1\ \Omega$.
3. Controller saturation windup during startup: verify anti-windup clamping prevents duty saturation windup.
4. Overfitting in ANFIS: verify checking/validation dataset is monitored during training with early stopping.
5. Incompatible FIS dimensions: verify input/output range and variable naming match deployment expectations.

---

### Task 1: Synchronize Averaged Plant Parameters and Physics
- [x] Inspect existing `boost_converter_openloop.slx` and `boost_converter_ode.m`.
- [ ] Update `boost_converter_params.m` to nominal $V_{in} = 20\text{ V}$, $V_{ref} = 40\text{ V}$, $L = 500\ \mu\text{H}$, $C = 1000\ \mu\text{F}$, $R = 500\ \Omega$, $r_L = 0.1\ \Omega$, $dt = 10\ \mu\text{s}$.
- [ ] Update `boost_converter_ode.m` to include the $- r_L \cdot i_L$ drop in the inductor equation.
- [ ] Verify steady-state values match the theoretical operating point ($V_{out} \approx 40\text{ V}, i_L \approx 0.16\text{ A}, D \approx 0.5$).

### Task 2: Implement Trajectory Data Generation Script
- [ ] Create `MATLAB_Simulink/04_Ideal_Controller_DataGen/generate_anfis_training_data.m`.
- [ ] Implement 24 diverse operating scenarios covering:
  - Nominal startup ($0 \to 40\text{V}$, $V_{in}=20\text{V}, R=500\ \Omega$)
  - Reference step up & down ($40\text{V} \to 45\text{V}, 40\text{V} \to 35\text{V}$)
  - Input voltage dips and swells ($20\text{V} \to 15\text{V}, 20\text{V} \to 25\text{V}, 18\text{V}, 22\text{V}$)
  - Load resistance steps ($500\ \Omega \to 250\ \Omega, 500\ \Omega \to 750\ \Omega$)
  - Combined line & load disturbances.
- [ ] Record $[run\_id, time\_s, Vin\_V, Vref\_V, Vout\_V, iL\_A, error\_V, error\_deriv\_Vps, duty\_target]$.
- [ ] Save raw data to:
  - `Data/Raw_SimData/anfis_reference_trajectories.mat`
  - `Data/Raw_SimData/anfis_reference_trajectories.csv`
- [ ] Generate reference trajectory plots into `Results_Plots/Step_Response_Graphs/`.

### Task 3: Implement Dataset Preparation Script (Splitting & Packaging)
- [ ] Create `MATLAB_Simulink/05_ANFIS_Training/prepare_anfis_datasets.m`.
- [ ] Load raw trajectories from `Data/Raw_SimData/anfis_reference_trajectories.mat`.
- [ ] Partition scenarios by `run_id` into:
  - Training: ~70% of runs
  - Validation: ~15% of runs
  - Testing: ~15% of runs
- [ ] Save CSV datasets:
  - `Data/Training_Datasets/anfis_training_data.csv`
  - `Data/Training_Datasets/anfis_validation_data.csv`
  - `Data/Training_Datasets/anfis_test_data.csv`
  - `Data/Training_Datasets/anfis_dataset_summary.csv`
- [ ] Save full dataset metadata to `Data/Training_Datasets/anfis_dataset_metadata.json`.

### Task 4: Implement Automated Dataset Validation Script
- [ ] Create `MATLAB_Simulink/05_ANFIS_Training/validate_anfis_dataset.m`.
- [ ] Perform programmatic validation checks:
  - Dimension consistency & column presence
  - Absence of NaN, Inf, or empty values
  - Target duty cycle strictly bounded within $[0.05, 0.95]$
  - Run-ID independence (zero leakage across train, val, and test splits)
  - Plausible physics ($V_{out} > 0$, $i_L \ge 0$, steady-state match)
  - Statistical summary (min, max, mean, std)
- [ ] Generate data distribution plots in `Results_Plots/Comparison_Tables/`.
- [ ] Write detailed report to `Data/Results/anfis_dataset_validation_report.txt` with PASS/FAIL status.

### Task 5: Implement ANFIS Training & Evaluation Script
- [ ] Create `MATLAB_Simulink/05_ANFIS_Training/train_anfis_from_dataset.m`.
- [ ] Train Sugeno FIS using grid partitioning (`genfisOptions('GridPartition', 'NumMembershipFunctions', [5 5])`) and hybrid learning (`anfisOptions`).
- [ ] Validate against held-out validation set and evaluate final test performance on test set.
- [ ] Save trained FIS to:
  - `MATLAB_Simulink/06_ANFIS_Deployment/anfis_trained.fis`
- [ ] Export training and test metrics to:
  - `Data/Results/anfis_training_metrics.csv`
  - `Data/Results/anfis_test_metrics.csv`
- [ ] Generate loss convergence plot, predicted vs actual duty plot, and residual error plot in `Results_Plots/PID_vs_Fuzzy_vs_ANFIS/`.

### Task 6: Documentation and Workflow Guide
- [ ] Create `MATLAB_Simulink/05_ANFIS_Training/README.md` detailing script run order, API usage, and reproducibility instructions.
- [ ] Execute all scripts in sequence to generate and validate all deliverables.
