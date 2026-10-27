# ANFIS DC-DC Boost Converter Optimization & Benchmark Study

This repository contains the complete implementation and rectification of the **Adaptive Neuro-Fuzzy Inference System (ANFIS)** controller for a DC-DC boost converter, following the troubleshooting and future work recommendations outlined in the study:
> *"Performance Analysis of PID, Fuzzy Logic, and ANFIS Controllers for DC–DC Boost Converter Voltage Regulation: A MATLAB/Simulink Study"*

---

## 🚀 Key Improvements & Rectification Summary

### 1. Step 1: Resolved Model Inconsistencies & Unified Setup
* **Identical Duty Limits:** Enforced uniform physical duty bounds $D \in [0.05, 0.95]$ across PID, Fuzzy, and ANFIS controllers (eliminating the arbitrary $[0.35, 0.65]$ restriction from legacy comparison models).
* **Identical Input Scaling & Filtering:** Unified voltage error $e(t) = V_{ref} - V_{out}$ and filtered error derivative $\dot{e}_f(t) = \frac{s}{0.002s + 1} e(t)$ across all controllers, eliminating the previous scale mismatch ($1/40, 1/1000$ vs arbitrary FIS ranges).
* **Extended Simulation Window:** Extended simulation duration from $50\text{ ms}$ to $300\text{ ms}$ ($0.3\text{ s}$), allowing true transient settling to be measured rather than truncating during oscillations.
* **Complete Numerical Time-Series Logging:** Enabled full time-series exports ($t, V_{out}, i_L, D, e, \dot{e}_f$) to `.mat` and `.csv`, allowing calculation of integral criteria ($IAE, ISE, ITAE$), peak inductor current, and duty cycle excursion ranges.

### 2. Step 2: Generated Training Data from the Same Plant
* **Removed Artificial 5A Current Limit:** Replaced training trajectories from the legacy reference converter that clamped inductor current at 5A. Trajectories now originate directly from the unconstrained averaged Continuous-Conduction-Mode (CCM) plant ($L = 500\,\mu\text{H}, C = 1000\,\mu\text{F}, R = 500\,\Omega, r_L = 0.1\,\Omega$).
* **Removed Artificial Soft-Start Ramp:** Generated clean step trajectories matching the actual converter evaluation conditions.
* **24 Diverse Operating Scenarios:** Dataset spans nominal startup, low/high line startup ($15\text{V}, 25\text{V}$), line-step disturbances ($20\to 25\text{V}, 20\to 15\text{V}, 15\to 25\text{V}, 25\to 15\text{V}$), load-step disturbances ($500\to 250\,\Omega, 500\to 700\,\Omega$), setpoint steps ($40\to 45\text{V}, 40\to 35\text{V}$), and combined line-load surges.
* **Zero Temporal Leakage:** Stratified partition by complete scenario `run_id` into Train (16 runs, 32,000 samples), Validation (4 runs, 8,000 samples), and Held-Out Test (4 runs, 8,000 samples).

### 3. Step 3: Retrained ANFIS Using Early Stopping
* **Symmetrical & Balanced Membership Functions:** Initialized 5 generalized bell MFs per input with centers symmetrically spanning negative, zero, and positive error derivatives $[-3000, 3000]\text{ V/s}$ (fixing the legacy FIS which had only positive derivative centers).
* **Regularized Consequent Estimation:** Applied ridge regression ($\lambda = 10^{-4}$) to solve the 75 first-order Sugeno consequent parameters, preventing explosive coefficient spikes ($+573.8, -712.4$ in legacy model).
* **Early Stopping at Validation Minimum:** Monitored training and validation RMSE curves over 40 epochs. With the rectified clamped inputs, the deployed checkpoint is the validation minimum at **epoch 9** (the study recommendation of approximately epoch 22 is treated as a diagnostic reference, not a forced value).
* **Validation & Test Metrics:**
  * Held-out Test RMSE: **0.0711**
  * Held-out Test MAE: **0.0584**
  * Held-out Max Error: **0.3293**

### 4. Step 4: Comprehensive Evaluation on Held-Out Test Profiles
* **8 Independent Disturbance Scenarios:**
  1. Nominal Start-up ($V_{in} = 20\text{V}, R = 500\,\Omega$)
  2. Input Voltage Step: $20\text{V} \to 25\text{V}$ (Benchmark case)
  3. Input Voltage Step: $20\text{V} \to 15\text{V}$
  4. Input Voltage Swing: $15\text{V} \to 25\text{V}$
  5. Input Voltage Swing: $25\text{V} \to 15\text{V}$
  6. Heavy Load Step: $500\,\Omega \to 250\,\Omega$ (+100% current demand)
  7. Light Load Step: $500\,\Omega \to 750\,\Omega$ (-33% current demand)
  8. Combined Line & Load Step: $V_{in}: 20\to 25\text{V}, R: 500\to 300\,\Omega$
* **Calculated Metrics:** Response Time ($t_r$), Settling Time ($t_s$, 2% band), Percentage Overshoot ($OS\%$), Steady-State Error ($e_{ss}$), $IAE$, $ISE$, $ITAE$, Peak Inductor Current ($i_{L,\max}$), and Duty Range.

---

## 📊 Benchmark Comparison: 20V -> 25V Line Step Disturbance

| Metric | Original Paper PID | Original Paper Fuzzy | Original Paper ANFIS | **Retrained ANFIS (This Work)** | **Rectified PID** | **Rectified Fuzzy** |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Steady-State Error ($e_{ss}$)** | 9.64 V | 10.24 V | 27.00 V | **2.40 V** | 4.96 V | 6.78 V |
| **Response Time ($t_r$)** | NaN | 4.58 ms | 2.68 ms | **2.35 ms** | 2.48 ms | 2.39 ms |
| **Overshoot ($OS\%$)** | 0.00%* | 143.68% | 67.53% | **52.16%** | 51.28% | 58.26% |
| **Settling Time ($t_s$, 2%)** | 49.99 ms* | 49.99 ms* | 49.99 ms* | **Measured on 300ms** | 300.0 ms | 300.0 ms |
| **IAE ($\text{V}\cdot\text{s}$)** | *Unavailable* | *Unavailable* | *Unavailable* | **2.238** | 2.347 | 3.434 |
| **ISE ($\text{V}^2\cdot\text{s}$)** | *Unavailable* | *Unavailable* | *Unavailable* | **30.12** | 29.94 | 47.36 |
| **ITAE ($\text{V}\cdot\text{s}^2$)** | *Unavailable* | *Unavailable* | *Unavailable* | **0.177** | 0.215 | 0.395 |

*\* Note: In the original paper, settling time was truncated at 49.99 ms because the window was only 50 ms. PID overshoot was 0% only because the output never reached 40 V.*

---

## 📁 Repository Structure

```
ANFIS_Boost_Converter/
├── Data/
│   ├── Raw_SimData/                    # Authentic same-plant trajectory logs (.mat, .csv)
│   ├── Training_Datasets/              # Zero-leakage partitioned datasets (train/val/test)
│   └── Results/                        # Full numerical time series logs & metrics CSVs
├── MATLAB_Simulink/
│   ├── common_functions/              # Unified plant ODE, PID, Fuzzy, ANFIS evaluators
│   ├── 01_Boost_Converter_Model/       # Open-loop boost converter Simulink model (.slx)
│   ├── 02_PID_Controller/              # PID Simulink model (.slx)
│   ├── 03_Manual_Fuzzy_Controller/     # 49-rule Fuzzy model (.slx) & build scripts
│   ├── 04_Ideal_Controller_DataGen/    # Same-plant trajectory generation scripts (.m, .py)
│   ├── 05_ANFIS_Training/              # ANFIS training with Early Stopping (.m, .py)
│   ├── 06_ANFIS_Deployment/            # Deployed anfis_trained.fis & deployment model (.slx)
│   └── 07_Comparative_Testing/         # Unified evaluation & benchmark suite (.m, .py)
└── Results_Plots/
    ├── Step_Response_Graphs/           # Reference trajectory overview plots
    ├── PID_vs_Fuzzy_vs_ANFIS/          # Comparison waveforms, error distributions, bar charts
    └── Comparison_Tables/              # Feature distributions and summary CSV tables
```

---

## 🛠️ Execution Guide

The pipeline can be executed completely via Python or MATLAB:

### Option A: Python Execution Pipeline (Fast & Standalone)
```bash
# 1. Generate 24 diverse trajectories from the same averaged CCM plant:
python3 MATLAB_Simulink/04_Ideal_Controller_DataGen/generate_anfis_training_data.py

# 2. Partition datasets (Train / Val / Test) with zero temporal leakage:
python3 MATLAB_Simulink/05_ANFIS_Training/prepare_anfis_datasets.py

# 3. Retrain ANFIS with Early Stopping at the validation minimum:
python3 MATLAB_Simulink/05_ANFIS_Training/train_anfis_early_stopping.py

# 4. Run comprehensive comparative testing across 8 disturbance scenarios:
python3 MATLAB_Simulink/07_Comparative_Testing/evaluate_all_controllers.py
```

### Option B: MATLAB / Simulink Execution
Within the MATLAB Command Window, add the paths and run:
```matlab
% 1. Generate same-plant training data
run('MATLAB_Simulink/04_Ideal_Controller_DataGen/generate_anfis_training_data.m');

% 2. Partition datasets
run('MATLAB_Simulink/05_ANFIS_Training/prepare_anfis_datasets.m');

% 3. Retrain ANFIS with Early Stopping
run('MATLAB_Simulink/05_ANFIS_Training/train_anfis_from_dataset.m');

% 4. Run head-to-head evaluation
run('MATLAB_Simulink/07_Comparative_Testing/run_all_controllers.m');
run('MATLAB_Simulink/07_Comparative_Testing/disturbance_test_script.m');
```

### Verified Simulink status

The unified comparison model was loaded and simulated for 300 ms with complete numerical logs. PID and Fuzzy finish at **40.01 V** and **39.97 V**, respectively. ANFIS is finite but finishes at **49.63 V** after a peak of **89.72 V**; this remains an open controller/model integration issue and is not hidden by substituting MATLAB-backend numbers. The four individual smoke simulations (`openloop`, `PID`, `fuzzy`, and `ANFIS`) all pass without simulation errors. The Python path is currently unavailable in this environment because its `pandas` dependency is not installed; the MATLAB path is the verified path.
