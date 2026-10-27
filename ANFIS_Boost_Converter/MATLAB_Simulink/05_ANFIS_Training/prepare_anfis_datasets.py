#!/usr/bin/env python3
"""
PREPARE_ANFIS_DATASETS (Python executable)
Partitions raw trajectory data by run_id into Train (16 runs), Validation (4 runs),
and Held-Out Test (4 runs) to prevent temporal data leakage.
Exports CSVs, summary statistics, JSON metadata, and distribution plots.
"""

import os
import json
import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.abspath(os.path.join(base_dir, '..', '..'))

    raw_csv = os.path.join(project_root, 'Data', 'Raw_SimData', 'anfis_reference_trajectories.csv')
    train_dir = os.path.join(project_root, 'Data', 'Training_Datasets')
    plots_dir = os.path.join(project_root, 'Results_Plots', 'Comparison_Tables')

    os.makedirs(train_dir, exist_ok=True)
    os.makedirs(plots_dir, exist_ok=True)

    df = pd.read_csv(raw_csv)
    num_runs = df['run_id'].nunique()
    print(f"=== Preparing ANFIS Datasets from {num_runs} Trajectories ({len(df)} samples) ===")

    train_runs = [1, 2, 4, 6, 7, 8, 10, 11, 12, 13, 16, 17, 19, 21, 22, 23] # 16 runs (~67%)
    val_runs   = [3, 9, 14, 20]                                             # 4 runs (~17%)
    test_runs  = [5, 15, 18, 24]                                            # 4 runs (~17%)

    # Check partition integrity
    all_runs = sorted(train_runs + val_runs + test_runs)
    assert all_runs == sorted(list(df['run_id'].unique())), "Partition must be mutually exclusive and exhaustive!"

    train_df = df[df['run_id'].isin(train_runs)].copy()
    val_df   = df[df['run_id'].isin(val_runs)].copy()
    test_df  = df[df['run_id'].isin(test_runs)].copy()

    print(f"Dataset Splits:")
    print(f"  Train:      {len(train_df)} samples across {len(train_runs)} runs")
    print(f"  Validation: {len(val_df)} samples across {len(val_runs)} runs")
    print(f"  Test:       {len(test_df)} samples across {len(test_runs)} runs")

    # Save CSVs
    train_csv_path = os.path.join(train_dir, 'anfis_training_data.csv')
    val_csv_path   = os.path.join(train_dir, 'anfis_validation_data.csv')
    test_csv_path  = os.path.join(train_dir, 'anfis_test_data.csv')

    train_df.to_csv(train_csv_path, index=False)
    val_df.to_csv(val_csv_path, index=False)
    test_df.to_csv(test_csv_path, index=False)

    print(f"Saved: {train_csv_path}")
    print(f"Saved: {val_csv_path}")
    print(f"Saved: {test_csv_path}")

    # Summary table
    vars_list = ['Vin_V', 'Vref_V', 'Vout_V', 'iL_A', 'error_V', 'error_deriv_Vps', 'duty_target']
    summary_rows = []
    for split_name, s_df in [('Train', train_df), ('Validation', val_df), ('Test', test_df)]:
        for v in vars_list:
            col = s_df[v]
            summary_rows.append({
                'Split': split_name,
                'Variable': v,
                'Min': col.min(),
                'Max': col.max(),
                'Mean': col.mean(),
                'StdDev': col.std()
            })
    summary_df = pd.DataFrame(summary_rows)
    summary_csv_path = os.path.join(train_dir, 'anfis_dataset_summary.csv')
    summary_df.to_csv(summary_csv_path, index=False)
    print(f"Saved: {summary_csv_path}")

    # Metadata JSON
    metadata = {
        'projectName': 'ANFIS-Controlled DC-DC Boost Converter',
        'modelType': 'Averaged Continuous-Conduction-Mode (CCM)',
        'creationDate': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
        'plantParameters': {
            'Vin_nom_V': 20.0,
            'Vref_nom_V': 40.0,
            'L_H': 500e-6,
            'C_F': 1000e-6,
            'R_Ohm': 500.0,
            'rL_Ohm': 0.1,
            'duty_min': 0.05,
            'duty_max': 0.95,
            'tau_f_s': 0.002
        },
        'splits': {
            'train': {'numRuns': len(train_runs), 'runIds': train_runs, 'samples': len(train_df)},
            'validation': {'numRuns': len(val_runs), 'runIds': val_runs, 'samples': len(val_df)},
            'test': {'numRuns': len(test_runs), 'runIds': test_runs, 'samples': len(test_df)}
        },
        'anfisSignals': {
            'input1': {'name': 'error_V', 'description': 'Voltage error (Vref - Vout) in Volts'},
            'input2': {'name': 'error_deriv_Vps', 'description': 'Filtered error derivative (de/dt) in V/s'},
            'target': {'name': 'duty_target', 'description': 'Commanded duty cycle D in [0.05, 0.95]'}
        }
    }
    meta_path = os.path.join(train_dir, 'anfis_dataset_metadata.json')
    with open(meta_path, 'w') as f:
        json.dump(metadata, f, indent=2)
    print(f"Saved: {meta_path}")

    # Feature distribution plots
    fig, axs = plt.subplots(2, 2, figsize=(11, 7))

    axs[0, 0].hist(train_df['error_V'], bins=35, color='#1f77b4', edgecolor='none')
    axs[0, 0].set_title('Distribution: Voltage Error e(t)', fontsize=10, fontweight='bold')
    axs[0, 0].set_xlabel('Error (V)')
    axs[0, 0].set_ylabel('Count')
    axs[0, 0].grid(True, linestyle=':', alpha=0.6)

    axs[0, 1].hist(train_df['error_deriv_Vps'], bins=35, color='#ff7f0e', edgecolor='none')
    axs[0, 1].set_title('Distribution: Filtered Error Derivative de/dt', fontsize=10, fontweight='bold')
    axs[0, 1].set_xlabel('Derivative (V/s)')
    axs[0, 1].set_ylabel('Count')
    axs[0, 1].grid(True, linestyle=':', alpha=0.6)

    axs[1, 0].hist(train_df['duty_target'], bins=35, color='#2ca02c', edgecolor='none')
    axs[1, 0].set_title('Distribution: Commanded Duty Cycle D', fontsize=10, fontweight='bold')
    axs[1, 0].set_xlabel('Duty Cycle D in [0.05, 0.95]')
    axs[1, 0].set_ylabel('Count')
    axs[1, 0].grid(True, linestyle=':', alpha=0.6)

    sc = axs[1, 1].scatter(train_df['error_V'], train_df['duty_target'], c=train_df['Vin_V'], cmap='viridis', s=10, alpha=0.5)
    plt.colorbar(sc, ax=axs[1, 1], label='Vin (V)')
    axs[1, 1].set_title('State-Action Map: Error vs Duty Command', fontsize=10, fontweight='bold')
    axs[1, 1].set_xlabel('Error (V)')
    axs[1, 1].set_ylabel('Duty Cycle D')
    axs[1, 1].grid(True, linestyle=':', alpha=0.6)

    fig.tight_layout()
    dist_plot_path = os.path.join(plots_dir, 'anfis_dataset_distributions.png')
    fig.savefig(dist_plot_path, dpi=300)
    plt.close(fig)
    print(f"Saved distribution plot: {dist_plot_path}")
    print("=== Dataset Preparation Complete ===")

if __name__ == '__main__':
    main()
