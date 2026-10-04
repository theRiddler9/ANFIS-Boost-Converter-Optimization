%% RUN_ALL_CONTROLLERS
% Step 7: Head-to-head comparison of PID vs manual-Fuzzy vs ANFIS on
% identical disturbance tests. Produces the core result plots/tables for
% your paper.

clear; clc; close all;
addpath(fullfile('..', 'common_functions'));

p = boost_converter_params();

%% 1) Load controllers
pidData.Kp = 0.002;   % realistic (non-aggressive) PID baseline gains
pidData.Ki = 0.3;     % tune these to reasonably match your base paper's
pidData.Kd = 0.00002; % PID benchmark numbers

fuzzyFis = readfis(fullfile('..', '03_Manual_Fuzzy_Controller', 'fuzzy_49rules.fis'));
anfisFis = readfis(fullfile('..', '05_ANFIS_Training', 'anfis_model.fis'));

controllers = {
    'PID',   pidData;
    'FUZZY', fuzzyFis;
    'ANFIS', anfisFis;
};

%% 2) Define the disturbance test (matches the 15V -> 25V test in your doc)
Vin_profile = @(t) 20 + 5 * (t >= 0.02); % 20V -> 25V step at t=0.02s

%% 3) Run each controller and collect logs + metrics
results = struct();
figure; hold on;

for c = 1:size(controllers, 1)
    name = controllers{c, 1};
    data = controllers{c, 2};

    log = simulate_closed_loop(name, data, Vin_profile, p);
    results.(name).log = log;
    results.(name).metrics = compute_metrics(log, p);

    plot(log.t * 1000, log.vC, 'LineWidth', 1.5, 'DisplayName', name);
end

yline(p.Vref, '--k', 'DisplayName', 'Reference (350V)');
xlabel('Time (ms)'); ylabel('Output Voltage (V)');
title('Step Disturbance Response: 20V \rightarrow 25V Input');
legend('Location', 'southeast');
grid on;

saveas(gcf, fullfile('..', '..', 'Results_Plots', 'PID_vs_Fuzzy_vs_ANFIS', 'step_response_comparison.png'));

%% 4) Build comparison table
names = fieldnames(results);
T = table();
for i = 1:numel(names)
    m = results.(names{i}).metrics;
    T = [T; table(string(names{i}), m.responseTime_ms, m.overshoot_pct, ...
         m.settlingTime_ms, m.steadyStateErr_V, ...
         'VariableNames', {'Controller','ResponseTime_ms','Overshoot_pct', ...
                            'SettlingTime_ms','SteadyStateErr_V'})]; %#ok<AGROW>
end

disp(T);
writetable(T, fullfile('..', '..', 'Data', 'Results', 'comparison_results.csv'));
writetable(T, fullfile('..', '..', 'Results_Plots', 'Comparison_Tables', 'comparison_results.csv'));

%% 5) Save everything for the report
save(fullfile('..', '..', 'Data', 'Results', 'all_controller_logs.mat'), 'results');
fprintf('Done. Results saved to Data/Results and Results_Plots.\n');

%% Local metric computation function
function m = compute_metrics(log, p)
    vC = log.vC; t = log.t; Vref = p.Vref;

    % Overshoot: max value above reference, as % of reference
    m.overshoot_pct = max(0, (max(vC) - Vref) / Vref * 100);

    % Response time: time to first reach within 2% of reference after start
    tol = 0.02 * Vref;
    idx = find(abs(vC - Vref) <= tol, 1, 'first');
    if isempty(idx)
        m.responseTime_ms = NaN;
    else
        m.responseTime_ms = t(idx) * 1000;
    end

    % Settling time: time after which it STAYS within 2% band
    withinBand = abs(vC - Vref) <= tol;
    settleIdx = find(~withinBand, 1, 'last'); % last index it's OUTSIDE band
    if isempty(settleIdx)
        m.settlingTime_ms = 0;
    else
        m.settlingTime_ms = t(min(settleIdx + 1, numel(t))) * 1000;
    end

    % Steady-state error: average error over the last 10% of the run
    tailIdx = round(0.9 * numel(vC)):numel(vC);
    m.steadyStateErr_V = mean(abs(Vref - vC(tailIdx)));
end
