#!/usr/bin/env python3
"""
GENERATE_ANFIS_TRAINING_DATA (Python executable)
Generates supervised training and validation trajectories directly from the
exact unconstrained averaged boost converter model without artificial current limits.
"""

import os
import json
import numpy as np
import pandas as pd
import scipy.io as sio
import matplotlib.pyplot as plt

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(base_dir, '..', '..'))

    raw_dir   = os.path.join(project_root, 'Data', 'Raw_SimData')
    plots_dir = os.path.join(project_root, 'Results_Plots', 'Step_Response_Graphs')
    os.makedirs(raw_dir, exist_ok=True)
    os.makedirs(plots_dir, exist_ok=True)

    # Converter parameters
    Vin_nom = 20.0
    Vref_nom = 40.0
    L = 500e-6
    C = 1000e-6
    R_nom = 500.0
    rL = 0.1
    fsw = 50e3
    dt = 1e-5
    Tsim = 0.2 # 200 ms
    d_min = 0.05
    d_max = 0.95
    tau_f = 0.002
    alpha_f = tau_f / (tau_f + dt)
    beta_f  = 1.0 / (tau_f + dt)

    N = int(Tsim / dt)
    time_s = np.linspace(0, Tsim, N)

    # Reference controller gains (tuned to match plant dynamics without windup)
    Kp = 0.002
    Ki = 0.03
    Kd = 0.00001

    # 24 diverse scenarios
    scenarios = [
        (1,  'Nominal Startup',              lambda t: 20.0,                          lambda t: 40.0,                         lambda t: 500.0),
        (2,  'Low Input Startup (15V)',       lambda t: 15.0,                          lambda t: 40.0,                         lambda t: 500.0),
        (3,  'High Input Startup (25V)',      lambda t: 25.0,                          lambda t: 40.0,                         lambda t: 500.0),
        (4,  'Heavy Load Startup (250 Ohm)',  lambda t: 20.0,                          lambda t: 40.0,                         lambda t: 250.0),
        (5,  'Light Load Startup (750 Ohm)',  lambda t: 20.0,                          lambda t: 40.0,                         lambda t: 750.0),
        (6,  'Low Setpoint Startup (35V)',    lambda t: 20.0,                          lambda t: 35.0,                         lambda t: 500.0),
        (7,  'High Setpoint Startup (45V)',   lambda t: 20.0,                          lambda t: 45.0,                         lambda t: 500.0),
        (8,  'Vref Step Up (40V -> 45V)',     lambda t: 20.0,                          lambda t: 40.0 + 5.0*(t>=0.05),         lambda t: 500.0),
        (9,  'Vref Step Down (40V -> 35V)',   lambda t: 20.0,                          lambda t: 40.0 - 5.0*(t>=0.05),         lambda t: 500.0),
        (10, 'Vin Step Up (20V -> 25V)',      lambda t: 20.0 + 5.0*(t>=0.05),          lambda t: 40.0,                         lambda t: 500.0),
        (11, 'Vin Step Down (20V -> 15V)',    lambda t: 20.0 - 5.0*(t>=0.05),          lambda t: 40.0,                         lambda t: 500.0),
        (12, 'Vin Step Up Minor (20V->22V)',  lambda t: 20.0 + 2.0*(t>=0.05),          lambda t: 40.0,                         lambda t: 500.0),
        (13, 'Vin Step Down Minor (20V->18V)',lambda t: 20.0 - 2.0*(t>=0.05),          lambda t: 40.0,                         lambda t: 500.0),
        (14, 'Vin Swing Low-to-High (15->25)',lambda t: 15.0 + 10.0*(t>=0.05),         lambda t: 40.0,                         lambda t: 500.0),
        (15, 'Vin Swing High-to-Low (25->15)',lambda t: 25.0 - 10.0*(t>=0.05),         lambda t: 40.0,                         lambda t: 500.0),
        (16, 'Load Step Drop (500 -> 350)',   lambda t: 20.0,                          lambda t: 40.0,                         lambda t: 500.0 - 150.0*(t>=0.05)),
        (17, 'Load Step Rise (500 -> 700)',   lambda t: 20.0,                          lambda t: 40.0,                         lambda t: 500.0 + 200.0*(t>=0.05)),
        (18, 'Heavy Load Step (500 -> 250)',  lambda t: 20.0,                          lambda t: 40.0,                         lambda t: 500.0 - 250.0*(t>=0.05)),
        (19, 'Combined Dip (Vin 18V, R 400)', lambda t: 20.0 - 2.0*(t>=0.05),          lambda t: 40.0,                         lambda t: 500.0 - 100.0*(t>=0.05)),
        (20, 'Combined Surge(Vin 23V, R 600)',lambda t: 20.0 + 3.0*(t>=0.05),          lambda t: 40.0,                         lambda t: 500.0 + 100.0*(t>=0.05)),
        (21, 'Ramp Vref Tracking (40->44V)',  lambda t: 20.0,                          lambda t: 40.0 + 200.0*max(0,min(0.02,t-0.05)), lambda t: 500.0),
        (22, 'High Vin Light Load (24V, 650)',lambda t: 24.0,                          lambda t: 40.0,                         lambda t: 650.0),
        (23, 'Low Vin Heavy Load (16V, 350)', lambda t: 16.0,                          lambda t: 40.0,                         lambda t: 350.0),
        (24, 'Multi-step Vin (20->23->17V)',  lambda t: 20.0 + 3.0*(t>=0.04) - 6.0*(t>=0.10), lambda t: 40.0,                  lambda t: 500.0)
    ]

    num_scenarios = len(scenarios)
    print(f"=== Generating Trajectory Dataset Across {num_scenarios} Scenarios on Same Plant ===")

    all_rows = []
    logs = {}

    for run_id, scen_name, vin_fn, vref_fn, rload_fn in scenarios:
        x = np.array([0.0, 0.0]) # iL, vC
        int_err = 0.0
        prev_err = 0.0
        prev_der = 0.0

        t_arr = np.zeros(N)
        vin_arr = np.zeros(N)
        vref_arr = np.zeros(N)
        vout_arr = np.zeros(N)
        il_arr = np.zeros(N)
        err_arr = np.zeros(N)
        eder_arr = np.zeros(N)
        duty_arr = np.zeros(N)

        for k in range(N):
            t = time_s[k]
            Vin_k = vin_fn(t)
            Vref_k = vref_fn(t)
            R_k = rload_fn(t)

            iL_k, Vout_k = x
            e = Vref_k - Vout_k
            if k == 0:
                de_dt = 0.0
            else:
                de_dt = alpha_f * prev_der + beta_f * (e - prev_err)

            int_err += e * dt

            # Feedforward + PID
            d_ff = max(d_min, min(d_max, 1.0 - Vin_k / max(Vref_k, 1.0)))
            d_raw = d_ff + Kp * e + Ki * int_err + Kd * de_dt
            d_sat = max(d_min, min(d_max, d_raw))

            if (d_raw >= d_max and e > 0) or (d_raw <= d_min and e < 0):
                int_err -= e * dt

            # Exact averaged boost converter ODE (no 5A clamp!)
            diL = (Vin_k - (1.0 - d_sat) * Vout_k - rL * iL_k) / L
            dvC = ((1.0 - d_sat) * iL_k - Vout_k / R_k) / C
            x = x + dt * np.array([diL, dvC])

            x[0] = max(0.0, x[0])
            x[1] = max(0.0, x[1])

            t_arr[k] = t
            vin_arr[k] = Vin_k
            vref_arr[k] = Vref_k
            vout_arr[k] = Vout_k
            il_arr[k] = iL_k
            err_arr[k] = e
            eder_arr[k] = de_dt
            duty_arr[k] = d_sat

            prev_err = e
            prev_der = de_dt

            # Subsample at 10 kHz (every 10th sample)
            if k % 10 == 0:
                all_rows.append({
                    'run_id': run_id,
                    'time_s': t,
                    'Vin_V': Vin_k,
                    'Vref_V': Vref_k,
                    'Vout_V': Vout_k,
                    'iL_A': iL_k,
                    'error_V': e,
                    'error_deriv_Vps': de_dt,
                    'duty_target': d_sat
                })

        logs[run_id] = {
            'run_id': run_id,
            'name': scen_name,
            'time_s': t_arr,
            'Vin_V': vin_arr,
            'Vref_V': vref_arr,
            'Vout_V': vout_arr,
            'iL_A': il_arr,
            'error_V': err_arr,
            'error_deriv_Vps': eder_arr,
            'duty_target': duty_arr
        }
        print(f"  Run {run_id:02d}/{num_scenarios}: {scen_name:<34} -> Vout_end = {vout_arr[-1]:.2f} V, D_end = {duty_arr[-1]:.4f}")

    df_subsampled = pd.DataFrame(all_rows)
    print(f"\nSubsampled dataset (10 kHz): {len(df_subsampled)} rows across {num_scenarios} runs")

    # Save CSV and MAT
    csv_path = os.path.join(raw_dir, 'anfis_reference_trajectories.csv')
    df_subsampled.to_csv(csv_path, index=False)
    print(f"Saved CSV: {csv_path}")

    # Prepare MAT dict
    mat_dict = {
        'run_id': df_subsampled['run_id'].values,
        'time_s': df_subsampled['time_s'].values,
        'Vin_V': df_subsampled['Vin_V'].values,
        'Vref_V': df_subsampled['Vref_V'].values,
        'Vout_V': df_subsampled['Vout_V'].values,
        'iL_A': df_subsampled['iL_A'].values,
        'error_V': df_subsampled['error_V'].values,
        'error_deriv_Vps': df_subsampled['error_deriv_Vps'].values,
        'duty_target': df_subsampled['duty_target'].values
    }
    mat_path = os.path.join(raw_dir, 'anfis_reference_trajectories.mat')
    sio.savemat(mat_path, mat_dict)
    print(f"Saved MAT: {mat_path}")

    # Overview Plots
    fig, axs = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    key_runs = [1, 8, 10, 11, 16, 18, 20]

    for rid in key_runs:
        lg = logs[rid]
        t_ms = lg['time_s'] * 1000.0
        axs[0].plot(t_ms, lg['Vout_V'], lw=1.5, label=f"Run {rid}: {lg['name']}")
        axs[1].plot(t_ms, lg['duty_target'], lw=1.5, label=f"Run {rid}: {lg['name']}")
        axs[2].plot(t_ms, lg['iL_A'], lw=1.5, label=f"Run {rid}: {lg['name']}")

    axs[0].axhline(40.0, color='black', linestyle='--', lw=1.6, label='Reference (40V)')
    axs[0].set_ylabel('Vout (V)', fontsize=10)
    axs[0].set_title('Averaged Boost Plant: Output Voltage Trajectories Across Key Scenarios', fontsize=11, fontweight='bold')
    axs[0].grid(True, linestyle=':', alpha=0.6)
    axs[0].legend(fontsize=7, loc='lower right')

    axs[1].set_ylabel('Target Duty D', fontsize=10)
    axs[1].set_title('Reference Commanded Duty Cycle Target D in [0.05, 0.95]', fontsize=11, fontweight='bold')
    axs[1].set_ylim([0, 1])
    axs[1].grid(True, linestyle=':', alpha=0.6)

    axs[2].set_ylabel('iL (A)', fontsize=10)
    axs[2].set_xlabel('Time (ms)', fontsize=10)
    axs[2].set_title('Inductor Current Trajectories (Natural Peak, No 5A Clamp)', fontsize=11, fontweight='bold')
    axs[2].grid(True, linestyle=':', alpha=0.6)

    fig.tight_layout()
    overview_plot_path = os.path.join(plots_dir, 'reference_trajectories_overview.png')
    fig.savefig(overview_plot_path, dpi=300)
    plt.close(fig)
    print(f"Saved overview plot: {overview_plot_path}")
    print("=== Data Generation Complete ===")

if __name__ == '__main__':
    main()
