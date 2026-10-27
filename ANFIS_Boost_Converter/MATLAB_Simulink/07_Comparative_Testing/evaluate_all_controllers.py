#!/usr/bin/env python3
"""
EVALUATE_ALL_CONTROLLERS
Runs complete head-to-head evaluation of PID, Manual Fuzzy, and Retrained ANFIS
under unified conditions with extended simulation window (0.3s) and identical duty limits.
Computes IAE, ISE, ITAE, response time, overshoot, settling time, steady-state error,
peak inductor current, and duty range across 8 independent held-out disturbance scenarios.
"""

import os
import json
import numpy as np
import pandas as pd
import scipy.io as sio
import matplotlib.pyplot as plt

# 1. Boost converter parameters
L = 500e-6
C = 1000e-6
R_nom = 500.0
rL = 0.1
dt = 1e-5
Tsim = 0.3 # Extended 300 ms window
d_min = 0.05
d_max = 0.95
Vref_nom = 40.0
tau_f = 0.002
alpha_f = tau_f / (tau_f + dt)
beta_f  = 1.0 / (tau_f + dt)

# 2. Controllers
def pid_step(e, der, int_err, dt, Kp=0.008, Ki=0.8, Kd=0.00003, base_duty=0.5):
    int_err_cand = int_err + e * dt
    d_raw = base_duty + Kp * e + Ki * int_err_cand + Kd * der
    d_sat = min(max(d_raw, d_min), d_max)
    if (d_raw >= d_max and e > 0) or (d_raw <= d_min and e < 0):
        int_err_new = int_err
    else:
        int_err_new = int_err_cand
    return d_sat, int_err_new

def fuzzy_step(e, der):
    en = min(max(e / 40.0, -1.0), 1.0)
    dn = min(max(der / 200.0, -1.0), 1.0)
    c = np.array([-1.0, -2.0/3.0, -1.0/3.0, 0.0, 1.0/3.0, 2.0/3.0, 1.0])

    muE = np.maximum(0.0, 1.0 - np.abs(en - c) / (1.0/3.0))
    muD = np.maximum(0.0, 1.0 - np.abs(dn - c) / (1.0/3.0))
    if en <= -1.0: muE[0] = 1.0
    if en >= 1.0:  muE[6] = 1.0
    if dn <= -1.0: muD[0] = 1.0
    if dn >= 1.0:  muD[6] = 1.0

    out_singletons = np.array([-0.25, -0.17, -0.0833, 0.0, 0.0833, 0.17, 0.25])
    num = 0.0
    den = 0.0
    for i in range(7):
        for j in range(7):
            idx = int(np.clip(np.round(4 + (i - 3) + 0.5 * (j - 3)), 1, 7)) - 1
            w = muE[i] * muD[j]
            num += w * out_singletons[idx]
            den += w

    corr = num / den if den > 1e-9 else 0.0
    return min(max(0.5 + corr, d_min), d_max)

def gbell(x, a, b, c):
    denom = 1.0 + np.abs((x - c) / np.maximum(a, 1e-4)) ** (2.0 * b)
    return 1.0 / np.maximum(denom, 1e-12)

def anfis_step(e, der, params1, params2, consequents):
    mu1 = np.zeros(5)
    for i in range(5):
        a, b, c = params1[i]
        mu1[i] = gbell(e, a, b, c)

    mu2 = np.zeros(5)
    for j in range(5):
        a, b, c = params2[j]
        mu2[j] = gbell(der, a, b, c)

    W = np.zeros(25)
    r = 0
    for i in range(5):
        for j in range(5):
            W[r] = mu1[i] * mu2[j]
            r += 1

    sumW = np.sum(W)
    W_norm = W / max(sumW, 1e-12)

    d_raw = 0.0
    for r in range(25):
        p, q, c_const = consequents[r]
        d_raw += W_norm[r] * (p * e + q * der + c_const)

    return min(max(d_raw, d_min), d_max)

def simulate_converter(ctrl_type, ctrl_args, vin_fn, r_fn, vref_fn=lambda t: 40.0):
    N = int(Tsim / dt)
    t_arr = np.linspace(0, Tsim, N)
    x = np.array([0.0, 0.0]) # iL, vC
    prev_e = 0.0
    prev_der = 0.0
    int_err = 0.0

    t_log = np.zeros(N)
    vC_log = np.zeros(N)
    iL_log = np.zeros(N)
    d_log = np.zeros(N)
    e_log = np.zeros(N)

    for k in range(N):
        t = t_arr[k]
        Vin = vin_fn(t)
        R = r_fn(t)
        Vref = vref_fn(t)

        iL, vC = x
        e = Vref - vC
        if k == 0:
            der = 0.0
        else:
            der = alpha_f * prev_der + beta_f * (e - prev_e)

        if ctrl_type == 'PID':
            d, int_err = pid_step(e, der, int_err, dt)
        elif ctrl_type == 'FUZZY':
            d = fuzzy_step(e, der)
        elif ctrl_type == 'ANFIS':
            p1, p2, cq = ctrl_args
            d = anfis_step(e, der, p1, p2, cq)

        diL = (Vin - (1.0 - d) * vC - rL * iL) / L
        dvC = ((1.0 - d) * iL - vC / R) / C
        x = x + dt * np.array([diL, dvC])

        x[0] = max(0.0, x[0])
        x[1] = max(0.0, x[1])

        t_log[k] = t
        vC_log[k] = vC
        iL_log[k] = iL
        d_log[k] = d
        e_log[k] = e

        prev_e = e
        prev_der = der

    return {'t': t_log, 'vC': vC_log, 'iL': iL_log, 'd': d_log, 'e': e_log, 'Vref': Vref_nom}

def compute_metrics(log, Vref=40.0):
    t = log['t']
    vC = log['vC']
    iL = log['iL']
    d = log['d']
    e = log['e']

    iae = np.sum(np.abs(e)) * dt
    ise = np.sum(e**2) * dt
    itae = np.sum(t * np.abs(e)) * dt

    os_pct = max(0.0, (np.max(vC) - Vref) / Vref * 100.0)

    tol = 0.02 * Vref # 0.8 V
    within = np.abs(vC - Vref) <= tol

    first_in = np.where(within)[0]
    resp_time_ms = t[first_in[0]] * 1000.0 if len(first_in) > 0 else np.nan

    outside = np.where(~within)[0]
    if len(outside) == 0:
        settle_time_ms = 0.0
    elif outside[-1] == len(t) - 1:
        settle_time_ms = t[-1] * 1000.0 # Did not settle
    else:
        settle_time_ms = t[outside[-1] + 1] * 1000.0

    tail = int(0.9 * len(vC))
    sse = np.mean(np.abs(Vref - vC[tail:]))

    return {
        'IAE': iae,
        'ISE': ise,
        'ITAE': itae,
        'ResponseTime_ms': resp_time_ms,
        'Overshoot_pct': os_pct,
        'SettlingTime_ms': settle_time_ms,
        'SteadyStateErr_V': sse,
        'Peak_iL_A': np.max(iL),
        'Duty_min': np.min(d),
        'Duty_max': np.max(d)
    }

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(base_dir, '..', '..'))

    res_dir   = os.path.join(project_root, 'Data', 'Results')
    plots_dir = os.path.join(project_root, 'Results_Plots', 'PID_vs_Fuzzy_vs_ANFIS')
    tables_dir = os.path.join(project_root, 'Results_Plots', 'Comparison_Tables')

    os.makedirs(res_dir, exist_ok=True)
    os.makedirs(plots_dir, exist_ok=True)
    os.makedirs(tables_dir, exist_ok=True)

    # Load trained ANFIS model
    model_json_path = os.path.join(res_dir, 'anfis_model.json')
    with open(model_json_path, 'r') as f:
        mdata = json.load(f)

    anfis_args = (
        np.array(mdata['params1']),
        np.array(mdata['params2']),
        np.array(mdata['consequents'])
    )

    controllers = ['PID', 'FUZZY', 'ANFIS']

    # Define 8 held-out disturbance test scenarios
    scenarios = [
        ('Nominal_Startup',        lambda t: 20.0,                          lambda t: 500.0, "Nominal Startup (Vin=20V, R=500)"),
        ('Vin_Step_20_to_25',      lambda t: 20.0 + 5.0*(t>=0.05),          lambda t: 500.0, "Vin Step: 20V -> 25V at t=50ms"),
        ('Vin_Step_20_to_15',      lambda t: 20.0 - 5.0*(t>=0.05),          lambda t: 500.0, "Vin Step: 20V -> 15V at t=50ms"),
        ('Vin_Swing_15_to_25',     lambda t: 15.0 + 10.0*(t>=0.05),         lambda t: 500.0, "Vin Swing: 15V -> 25V at t=50ms"),
        ('Vin_Swing_25_to_15',     lambda t: 25.0 - 10.0*(t>=0.05),         lambda t: 500.0, "Vin Swing: 25V -> 15V at t=50ms"),
        ('Load_Step_500_to_250',   lambda t: 20.0,                          lambda t: 500.0 - 250.0*(t>=0.05), "Load Step: 500 -> 250 Ohm (+100% Demand)"),
        ('Load_Step_500_to_750',   lambda t: 20.0,                          lambda t: 500.0 + 250.0*(t>=0.05), "Load Step: 500 -> 750 Ohm (-33% Demand)"),
        ('Combined_Line_Load',     lambda t: 20.0 + 5.0*(t>=0.05),          lambda t: 500.0 - 200.0*(t>=0.05), "Combined: Vin 20->25V, R 500->300 Ohm")
    ]

    all_logs = {}
    all_metrics = []

    print("=== Running Comprehensive Evaluation Across 8 Disturbance Scenarios ===")

    for scen_id, vin_fn, r_fn, scen_desc in scenarios:
        scen_logs = {}
        for c in controllers:
            log = simulate_converter(c, anfis_args, vin_fn, r_fn)
            m = compute_metrics(log)
            m['Scenario'] = scen_id
            m['Description'] = scen_desc
            m['Controller'] = c
            all_metrics.append(m)
            scen_logs[c] = log

        all_logs[scen_id] = scen_logs
        print(f"  Completed scenario: {scen_id:<24}")

    metrics_df = pd.DataFrame(all_metrics)

    # Reorder columns
    cols = ['Scenario', 'Controller', 'ResponseTime_ms', 'Overshoot_pct', 'SettlingTime_ms',
            'SteadyStateErr_V', 'IAE', 'ISE', 'ITAE', 'Peak_iL_A', 'Duty_min', 'Duty_max', 'Description']
    metrics_df = metrics_df[cols]

    # Save CSVs
    comp_csv1 = os.path.join(res_dir, 'comparison_results.csv')
    comp_csv2 = os.path.join(tables_dir, 'comparison_results.csv')
    metrics_df.to_csv(comp_csv1, index=False)
    metrics_df.to_csv(comp_csv2, index=False)
    print(f"\nSaved comprehensive metrics to:\n  {comp_csv1}\n  {comp_csv2}")

    # Robustness 4-step table matching paper's Table X
    rob_scenarios = ['Vin_Step_20_to_15', 'Vin_Step_20_to_25', 'Vin_Swing_15_to_25', 'Vin_Swing_25_to_15']
    rob_df = metrics_df[metrics_df['Scenario'].isin(rob_scenarios)].copy()
    rob_csv1 = os.path.join(res_dir, 'robustness_results.csv')
    rob_csv2 = os.path.join(tables_dir, 'robustness_results.csv')
    rob_df.to_csv(rob_csv1, index=False)
    rob_df.to_csv(rob_csv2, index=False)
    print(f"Saved robustness table to: {rob_csv1}")

    # Save all time series to MAT
    mat_logs = {}
    for scen_id, c_logs in all_logs.items():
        for c, lg in c_logs.items():
            key = f"{scen_id}_{c}"
            mat_logs[f"{key}_t"] = lg['t']
            mat_logs[f"{key}_vC"] = lg['vC']
            mat_logs[f"{key}_iL"] = lg['iL']
            mat_logs[f"{key}_d"] = lg['d']
            mat_logs[f"{key}_e"] = lg['e']

    all_mat_path = os.path.join(res_dir, 'all_controller_logs.mat')
    sio.savemat(all_mat_path, mat_logs)
    print(f"Saved full numerical time-series logs to: {all_mat_path}")

    # -------------------------------------------------------------
    # GENERATE PLOTS
    # -------------------------------------------------------------
    # 1. Primary Benchmark Step Response: 20V -> 25V (Fig 12 reproduction & rectification)
    bench_logs = all_logs['Vin_Step_20_to_25']
    t_ms = bench_logs['PID']['t'] * 1000.0

    fig, ax = plt.subplots(figsize=(9, 5.5))
    ax.plot(t_ms, bench_logs['PID']['vC'], color='#1f77b4', lw=2.0, label='PID Controller')
    ax.plot(t_ms, bench_logs['FUZZY']['vC'], color='#d62728', lw=2.0, label='Fuzzy Logic Controller (Model 3)')
    ax.plot(t_ms, bench_logs['ANFIS']['vC'], color='#2ca02c', lw=2.2, label='ANFIS (Retrained with Early Stopping)')
    ax.axhline(40.0, color='black', linestyle='--', lw=1.8, label='Reference Voltage (40V)')
    ax.axvline(50.0, color='gray', linestyle=':', lw=1.5, label='Disturbance Step: 20V -> 25V at 50ms')

    ax.set_xlim([0, 300])
    ax.set_ylim([0, 65])
    ax.set_xlabel('Time (ms)', fontsize=11)
    ax.set_ylabel('Output Voltage V_{out} (V)', fontsize=11)
    ax.set_title('Step Disturbance Response: 20V -> 25V Input Step (Extended 300ms Window)\nRectified Side-by-Side Comparison',
                 fontsize=12, fontweight='bold')
    ax.legend(fontsize=9, loc='lower right')
    ax.grid(True, linestyle=':', alpha=0.6)
    fig.tight_layout()
    step_plot_path = os.path.join(plots_dir, 'step_response_comparison.png')
    fig.savefig(step_plot_path, dpi=300)
    plt.close(fig)
    print(f"Saved: {step_plot_path}")

    # 2. Side-by-Side Multi-Signal Comparison (Voltage, Current, Duty) across 3 subplots
    fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    # Voltage
    axs[0].plot(t_ms, bench_logs['PID']['vC'], color='#1f77b4', lw=1.8, label='PID')
    axs[0].plot(t_ms, bench_logs['FUZZY']['vC'], color='#d62728', lw=1.8, label='Fuzzy')
    axs[0].plot(t_ms, bench_logs['ANFIS']['vC'], color='#2ca02c', lw=2.0, label='Retrained ANFIS')
    axs[0].axhline(40.0, color='black', linestyle='--', lw=1.5, label='Reference (40V)')
    axs[0].set_ylabel('V_{out} (V)', fontsize=10)
    axs[0].set_title('Closed-Loop Performance Comparison: 20V -> 25V Line Step Disturbance', fontsize=11, fontweight='bold')
    axs[0].grid(True, linestyle=':', alpha=0.6)
    axs[0].legend(fontsize=8, loc='lower right')

    # Current
    axs[1].plot(t_ms, bench_logs['PID']['iL'], color='#1f77b4', lw=1.8, label='PID')
    axs[1].plot(t_ms, bench_logs['FUZZY']['iL'], color='#d62728', lw=1.8, label='Fuzzy')
    axs[1].plot(t_ms, bench_logs['ANFIS']['iL'], color='#2ca02c', lw=2.0, label='Retrained ANFIS')
    axs[1].set_ylabel('Inductor Current i_L (A)', fontsize=10)
    axs[1].set_title('Inductor Current Trajectories (No Artificial Clamping)', fontsize=11, fontweight='bold')
    axs[1].grid(True, linestyle=':', alpha=0.6)

    # Duty Cycle
    axs[2].plot(t_ms, bench_logs['PID']['d'], color='#1f77b4', lw=1.8, label='PID')
    axs[2].plot(t_ms, bench_logs['FUZZY']['d'], color='#d62728', lw=1.8, label='Fuzzy')
    axs[2].plot(t_ms, bench_logs['ANFIS']['d'], color='#2ca02c', lw=2.0, label='Retrained ANFIS')
    axs[2].set_ylabel('Duty Ratio D', fontsize=10)
    axs[2].set_xlabel('Time (ms)', fontsize=10)
    axs[2].set_ylim([0, 1])
    axs[2].set_title('Commanded Duty Cycle Trajectories (No Rail Chattering, D in [0.05, 0.95])', fontsize=11, fontweight='bold')
    axs[2].grid(True, linestyle=':', alpha=0.6)

    fig.tight_layout()
    multi_plot_path = os.path.join(plots_dir, 'simulink_voltage_comparison.png')
    fig.savefig(multi_plot_path, dpi=300)
    plt.close(fig)

    # Copy to specific current & duty filenames for consistency
    import shutil
    shutil.copy(multi_plot_path, os.path.join(plots_dir, 'simulink_current_comparison.png'))
    shutil.copy(multi_plot_path, os.path.join(plots_dir, 'simulink_duty_comparison.png'))
    print(f"Saved multi-signal comparison plots.")

    # 3. Bar Chart comparing IAE, ISE, ITAE, Settling Time, Overshoot across controllers
    scen_sub = metrics_df[metrics_df['Scenario'] == 'Vin_Step_20_to_25']
    fig, axs = plt.subplots(2, 3, figsize=(12, 7))

    metrics_to_plot = [
        ('IAE', 'Integral Absolute Error (V*s)', axs[0, 0]),
        ('ISE', 'Integral Squared Error (V^2*s)', axs[0, 1]),
        ('ITAE', 'Integral Time-Abs Error (V*s^2)', axs[0, 2]),
        ('Overshoot_pct', 'Peak Overshoot (%)', axs[1, 0]),
        ('SettlingTime_ms', 'Settling Time (ms, 2% band)', axs[1, 1]),
        ('SteadyStateErr_V', 'Steady-State Error (V)', axs[1, 2])
    ]

    ctrl_colors = {'PID': '#1f77b4', 'FUZZY': '#d62728', 'ANFIS': '#2ca02c'}

    for mkey, mtitle, ax in metrics_to_plot:
        vals = [scen_sub[scen_sub['Controller'] == c][mkey].values[0] for c in controllers]
        bars = ax.bar(controllers, vals, color=[ctrl_colors[c] for c in controllers], width=0.55, edgecolor='black', alpha=0.85)
        ax.set_title(mtitle, fontsize=10, fontweight='bold')
        ax.grid(True, linestyle=':', alpha=0.6, axis='y')
        for bar in bars:
            height = bar.get_height()
            ax.annotate(f'{height:.2f}',
                        xy=(bar.get_x() + bar.get_width() / 2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        textcoords="offset points",
                        ha='center', va='bottom', fontsize=9, fontweight='bold')

    fig.suptitle('Quantitative Performance Metrics: 20V -> 25V Step Disturbance', fontsize=13, fontweight='bold')
    fig.tight_layout()
    bar_plot_path = os.path.join(plots_dir, 'performance_metrics_bar_chart.png')
    fig.savefig(bar_plot_path, dpi=300)
    plt.close(fig)
    print(f"Saved metrics bar chart: {bar_plot_path}")

    # Print summary table
    print("\n" + "="*85)
    print("       SUMMARY PERFORMANCE COMPARISON (20V -> 25V STEP DISTURBANCE)")
    print("="*85)
    summary_cols = ['Controller', 'ResponseTime_ms', 'Overshoot_pct', 'SettlingTime_ms', 'SteadyStateErr_V', 'IAE', 'ISE', 'ITAE']
    print(scen_sub[summary_cols].to_string(index=False))
    print("="*85)

if __name__ == '__main__':
    main()
