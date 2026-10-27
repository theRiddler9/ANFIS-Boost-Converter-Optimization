%% RUN_ALL_CONTROLLERS
% Rectified head-to-head comparison of PID, Manual Fuzzy, and Retrained ANFIS
% on identical disturbance tests under unified duty limits [0.05, 0.95] and
% extended simulation window (0.3 s) with full numerical logging and metric computation.

clear; clc; close all;

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fullfile(scriptDir, '..', '..');
addpath(fullfile(projectRoot, 'MATLAB_Simulink', 'common_functions'));

p = boost_converter_params();
p.Tsim = 0.3; % 300 ms extended window

%% 1) Controller Setup
% PID Controller: tuned anti-windup PI
pidData.Kp = 0.008;
pidData.Ki = 0.8;
pidData.Kd = 0.00003;
pidData.base_duty = 0.5;

% Manual Fuzzy Controller: Model 3 7-term zero-order Sugeno
fuzzyData = [];

% Retrained ANFIS Controller: Early-stopped checkpoint
anfisModelPath = fullfile(projectRoot, 'Data', 'Results', 'anfis_model.mat');
if exist(anfisModelPath, 'file')
    loadedAnfis = load(anfisModelPath);
    anfisData = loadedAnfis.anfisModel;
else
    anfisFisPath = fullfile(projectRoot, 'MATLAB_Simulink', '06_ANFIS_Deployment', 'anfis_trained.fis');
    anfisData = anfisFisPath;
end

controllers = {
    'PID',   pidData;
    'FUZZY', fuzzyData;
    'ANFIS', anfisData;
};

%% 2) Define Disturbance Test (20V -> 25V step at t=0.05s)
Vin_profile = @(t) 20 + 5 * (t >= 0.05);

%% 3) Simulate Each Controller and Collect Full Numerical Logs + Metrics
results = struct();
fig = figure('Visible', 'off', 'Position', [100 100 900 550]);
hold on; grid on;

colors = {'#1f77b4', '#d62728', '#2ca02c'};

for c = 1:size(controllers, 1)
    name = controllers{c, 1};
    data = controllers{c, 2};

    log = simulate_closed_loop(name, data, Vin_profile, p);
    results.(name).log = log;
    results.(name).metrics = compute_performance_metrics(log, p);

    plot(log.t * 1000, log.vC, 'Color', colors{c}, 'LineWidth', 1.8, 'DisplayName', name);
end

yline(p.Vref, '--k', 'LineWidth', 1.5, 'DisplayName', 'Reference Voltage (40V)');
xline(50, ':k', 'LineWidth', 1.2, 'DisplayName', 'Disturbance Step (t=50ms)');
xlabel('Time (ms)', 'FontSize', 11);
ylabel('Output Voltage V_{out} (V)', 'FontSize', 11);
title('Step Disturbance Response: 20V \rightarrow 25V Input Step (Extended 300ms Window)', ...
    'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 9);
xlim([0, 300]);

stepPlotPath = fullfile(projectRoot, 'Results_Plots', 'PID_vs_Fuzzy_vs_ANFIS', 'step_response_comparison.png');
saveas(fig, stepPlotPath);
close(fig);
fprintf('Saved step response comparison plot: %s\n', stepPlotPath);

%% 4) Build Comprehensive Comparison Table
T = table();
for c = 1:size(controllers, 1)
    name = controllers{c, 1};
    m = results.(name).metrics;
    row = table(string(name), m.ResponseTime_ms, m.Overshoot_pct, m.SettlingTime_ms, ...
        m.SteadyStateErr_V, m.IAE, m.ISE, m.ITAE, m.Peak_iL_A, m.Duty_min, m.Duty_max, ...
        'VariableNames', {'Controller', 'ResponseTime_ms', 'Overshoot_pct', 'SettlingTime_ms', ...
                          'SteadyStateErr_V', 'IAE', 'ISE', 'ITAE', 'Peak_iL_A', 'Duty_min', 'Duty_max'});
    T = [T; row]; %#ok<AGROW>
end

disp('=== Head-to-Head Performance Comparison (20V -> 25V Step) ===');
disp(T);

csv1 = fullfile(projectRoot, 'Data', 'Results', 'comparison_results.csv');
csv2 = fullfile(projectRoot, 'Results_Plots', 'Comparison_Tables', 'comparison_results.csv');
writetable(T, csv1);
writetable(T, csv2);
fprintf('Saved comparison table to: %s\n', csv1);

%% 5) Save Numerical Time-Series Logs
allMatPath = fullfile(projectRoot, 'Data', 'Results', 'all_controller_logs.mat');
save(allMatPath, 'results', 'p');
fprintf('Saved complete numerical time-series logs: %s\n', allMatPath);
fprintf('=== run_all_controllers Execution Complete ===\n');
