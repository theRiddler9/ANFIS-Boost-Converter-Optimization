# ANFIS Dataset Generation, Validation & Training Pipeline

This folder contains the complete, reproducible pipeline to generate, validate, package, and train an Adaptive Neuro-Fuzzy Inference System (ANFIS) controller for the averaged continuous-conduction-mode (CCM) DC–DC boost converter.

---

## 1. Converter Parameters & Plant Equations

The plant follows the averaged non-ideal boost converter dynamics with inductor parasitic resistance ($r_L$):

$$\begin{aligned}
L \frac{di_L}{dt} &= V_{\text{in}} - (1-D) V_{\text{out}} - r_L i_L \\
C \frac{dV_{\text{out}}}{dt} &= (1-D) i_L - \frac{V_{\text{out}}}{R}
\end{aligned}$$

* **Nominal Input Voltage ($V_{\text{in}}$):** $20\text{ V}$ (swept over $15\text{V} - 25\text{V}$)
* **Reference Output Voltage ($V_{\text{ref}}$):** $40\text{ V}$ (swept over $35\text{V} - 45\text{V}$)
* **Boost Inductor ($L$):** $500\ \mu\text{H}$
* **Inductor Series Resistance ($r_L$):** $0.1\ \Omega$
* **Output Filter Capacitor ($C$):** $1000\ \mu\text{F}$
* **Load Resistance ($R$):** $500\ \Omega$ (swept over $250\ \Omega - 750\ \Omega$)
* **Nominal Operating Point:** $D \approx 0.5019$, $V_{\text{out}} \approx 40.0\text{ V}$, $i_L \approx 0.16\text{ A}$

---

## 2. Pipeline Execution Order

To regenerate the entire dataset, validate the numerical integrity, and train the ANFIS controller from scratch, run the scripts in this exact order:

```matlab
% Step 1: Generate diverse state-action trajectories from the verified plant
run('MATLAB_Simulink/04_Ideal_Controller_DataGen/generate_anfis_training_data.m');

% Step 2: Clean, format, and split into Train / Validation / Test sets by complete Run ID
run('MATLAB_Simulink/05_ANFIS_Training/prepare_anfis_datasets.m');

% Step 3: Run comprehensive physical & numerical automated validation suite
run('MATLAB_Simulink/05_ANFIS_Training/validate_anfis_dataset.m');

% Step 4: Train Sugeno ANFIS via hybrid learning & evaluate on held-out test data
run('MATLAB_Simulink/05_ANFIS_Training/train_anfis_from_dataset.m');
```

---

## 3. Script Responsibilities & Roles

| Script | Location | Key Outputs |
| :--- | :--- | :--- |
| `generate_anfis_training_data.m` | `MATLAB_Simulink/04_Ideal_Controller_DataGen/` | `Data/Raw_SimData/anfis_reference_trajectories.mat`<br>`Data/Raw_SimData/anfis_reference_trajectories.csv`<br>`Results_Plots/Step_Response_Graphs/reference_trajectories_overview.png` |
| `prepare_anfis_datasets.m` | `MATLAB_Simulink/05_ANFIS_Training/` | `Data/Training_Datasets/anfis_training_data.csv`<br>`Data/Training_Datasets/anfis_validation_data.csv`<br>`Data/Training_Datasets/anfis_test_data.csv`<br>`Data/Training_Datasets/anfis_dataset_summary.csv`<br>`Data/Training_Datasets/anfis_dataset_metadata.json` |
| `validate_anfis_dataset.m` | `MATLAB_Simulink/05_ANFIS_Training/` | `Data/Results/anfis_dataset_validation_report.txt`<br>`Results_Plots/Comparison_Tables/anfis_dataset_distributions.png` |
| `train_anfis_from_dataset.m` | `MATLAB_Simulink/05_ANFIS_Training/` | `MATLAB_Simulink/06_ANFIS_Deployment/anfis_trained.fis`<br>`Data/Results/anfis_training_metrics.csv`<br>`Data/Results/anfis_test_metrics.csv`<br>`Results_Plots/PID_vs_Fuzzy_vs_ANFIS/anfis_loss_convergence.png`<br>`Results_Plots/PID_vs_Fuzzy_vs_ANFIS/anfis_pred_vs_target_test.png`<br>`Results_Plots/PID_vs_Fuzzy_vs_ANFIS/anfis_residual_error_test.png` |

---

## 4. ANFIS Controller Signal Specification

* **Input 1 (`error_V`):** $e(k) = V_{\text{ref}} - V_{\text{out}}$ (Volts)
* **Input 2 (`error_diff_V`):** $\Delta e(k) = e(k) - e(k-1)$ (Volts per sample)
* **Target / Output (`duty_target`):** Commanded duty cycle $D \in [0.05, 0.95]$
* **FIS Architecture:** First-order Sugeno system with $5 \times 5 = 25$ rules and generalized bell membership functions (`gbellmf`).
* **Training Method:** Hybrid optimization combining Least-Squares estimation for linear output coefficients and Gradient Descent backpropagation for input membership parameters.

---

## 5. Verification & Anti-Leakage Guarantee

* **Whole-Run Partitioning:** Data splitting is strictly partitioned by complete simulation runs (`run_id`), preventing temporal cross-contamination between adjacent time steps.
* **Checks:**
  1. No `NaN` or `Inf` values across any column.
  2. Commanded duty strictly bounded in $[0.05, 0.95]$.
  3. Plausible plant states ($V_{\text{out}} \ge 0$, $i_L \ge 0$).
  4. Zero overlap across Train ($16$ runs, $8000$ samples), Validation ($4$ runs, $2000$ samples), and Test ($4$ runs, $2000$ samples) splits.
