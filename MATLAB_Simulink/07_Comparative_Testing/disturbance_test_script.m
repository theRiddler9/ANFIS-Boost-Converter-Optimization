%% DISTURBANCE_TEST_SCRIPT
% Runs the PID / Fuzzy / ANFIS comparison across MULTIPLE disturbance
% scenarios (not just the single 20V->25V step in run_all_controllers.m),
% to show robustness across the operating range. Produces one summary
% table across all scenarios -- useful for a "robustness" section/table
% in your paper.

clear; clc; close all;
addpath(fullfile('..', 'common_functions'));

p = boost_converter_params();

pidData.Kp = 0.002; pidData.Ki = 0.3; pidData.Kd = 0.00002;
fuzzyFis = readfis(fullfile('..', '03_Manual_Fuzzy_Controller', 'fuzzy_49rules.fis'));
anfisFis = readfis(fullfile('..', '05_ANFIS_Training', 'anfis_model.fis'));

controllers = {'PID', pidData; 'FUZZY', fuzzyFis; 'ANFIS', anfisFis};

% Scenarios: [Vin_start, Vin_end, step_time]
scenarios = [
    20 15 0.02;
    20 25 0.02;
    15 25 0.02;
    25 15 0.02;
];

allResults = table();

for s = 1:size(scenarios, 1)
    Vstart = scenarios(s,1); Vend = scenarios(s,2); tstep = scenarios(s,3);
    Vin_profile = @(t) Vstart + (Vend - Vstart) * (t >= tstep);

    for c = 1:size(controllers, 1)
        name = controllers{c, 1};
        data = controllers{c, 2};

        log = simulate_closed_loop(name, data, Vin_profile, p);
        m = local_compute_metrics(log, p);

        row = table(string(name), Vstart, Vend, m.responseTime_ms, ...
            m.overshoot_pct, m.settlingTime_ms, m.steadyStateErr_V, ...
            'VariableNames', {'Controller','Vin_start','Vin_end', ...
            'ResponseTime_ms','Overshoot_pct','SettlingTime_ms','SteadyStateErr_V'});
        allResults = [allResults; row]; %#ok<AGROW>
    end
    fprintf('Scenario %d/%d complete (%dV -> %dV)\n', s, size(scenarios,1), Vstart, Vend);
end

disp(allResults);
writetable(allResults, fullfile('..', '..', 'Data', 'Results', 'robustness_results.csv'));
writetable(allResults, fullfile('..', '..', 'Results_Plots', 'Comparison_Tables', 'robustness_results.csv'));
fprintf('Saved robustness comparison table.\n');

function m = local_compute_metrics(log, p)
    vC = log.vC; t = log.t; Vref = p.Vref;
    m.overshoot_pct = max(0, (max(vC) - Vref) / Vref * 100);
    tol = 0.02 * Vref;
    idx = find(abs(vC - Vref) <= tol, 1, 'first');
    m.responseTime_ms = NaN; if ~isempty(idx), m.responseTime_ms = t(idx)*1000; end
    withinBand = abs(vC - Vref) <= tol;
    settleIdx = find(~withinBand, 1, 'last');
    m.settlingTime_ms = 0; if ~isempty(settleIdx), m.settlingTime_ms = t(min(settleIdx+1,numel(t)))*1000; end
    tailIdx = round(0.9*numel(vC)):numel(vC);
    m.steadyStateErr_V = mean(abs(Vref - vC(tailIdx)));
end
