%% DISTURBANCE_TEST_SCRIPT
% Evaluates PID, Fuzzy, and Retrained ANFIS across multiple disturbance
% scenarios (input-voltage steps, input swings, and load steps) over an
% extended 0.3 s window with complete metric calculation (IAE, ISE, ITAE, etc.).

clear; clc; close all;

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fullfile(scriptDir, '..', '..');
addpath(fullfile(projectRoot, 'MATLAB_Simulink', 'common_functions'));

p = boost_converter_params();
p.Tsim = 0.3; % 300 ms extended duration

%% 1) Controllers Setup
pidData.Kp = 0.008;
pidData.Ki = 0.8;
pidData.Kd = 0.00003;
pidData.base_duty = 0.5;

fuzzyData = [];

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

%% 2) Define 4 Key Robustness Input-Voltage Scenarios (matching Table X in paper)
% [Vstart, Vend, step_time]
scenarios = [
    20, 15, 0.05;
    20, 25, 0.05;
    15, 25, 0.05;
    25, 15, 0.05;
];

allResults = table();

for s = 1:size(scenarios, 1)
    Vstart = scenarios(s, 1);
    Vend   = scenarios(s, 2);
    tstep  = scenarios(s, 3);
    Vin_profile = @(t) Vstart + (Vend - Vstart) * (t >= tstep);

    for c = 1:size(controllers, 1)
        name = controllers{c, 1};
        data = controllers{c, 2};

        log = simulate_closed_loop(name, data, Vin_profile, p);
        m = compute_performance_metrics(log, p);

        row = table(string(name), Vstart, Vend, m.ResponseTime_ms, ...
            m.Overshoot_pct, m.SettlingTime_ms, m.SteadyStateErr_V, ...
            m.IAE, m.ISE, m.ITAE, m.Peak_iL_A, m.Duty_min, m.Duty_max, ...
            'VariableNames', {'Controller', 'Vin_start', 'Vin_end', ...
            'ResponseTime_ms', 'Overshoot_pct', 'SettlingTime_ms', 'SteadyStateErr_V', ...
            'IAE', 'ISE', 'ITAE', 'Peak_iL_A', 'Duty_min', 'Duty_max'});
        allResults = [allResults; row]; %#ok<AGROW>
    end
    fprintf('Scenario %d/%d complete (%dV -> %dV)\n', s, size(scenarios, 1), Vstart, Vend);
end

disp('=== Robustness Across Input-Voltage Scenarios ===');
disp(allResults);

csv1 = fullfile(projectRoot, 'Data', 'Results', 'robustness_results.csv');
csv2 = fullfile(projectRoot, 'Results_Plots', 'Comparison_Tables', 'robustness_results.csv');
writetable(allResults, csv1);
writetable(allResults, csv2);
fprintf('Saved robustness results to: %s\n', csv1);
