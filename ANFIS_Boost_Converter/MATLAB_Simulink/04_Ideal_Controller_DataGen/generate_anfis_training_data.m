%% GENERATE_ANFIS_TRAINING_DATA
% Generates an authentic, physically consistent supervised dataset directly
% from the EXACT same averaged CCM boost converter plant used in evaluation.
%
% Rectifications applied per study recommendations:
%   1. Removed artificial 5A inductor current limit.
%   2. Removed artificial soft-start ramp to match true plant step response.
%   3. Diverse operational profiles: startup transients, line-step disturbances,
%      load-step changes, and setpoint tracking.
%   4. Unified derivative filtering: s / (0.002s + 1).
%   5. Identical duty cycle saturation: [0.05, 0.95].
%
% Outputs:
%   - Data/Raw_SimData/anfis_reference_trajectories.mat
%   - Data/Raw_SimData/anfis_reference_trajectories.csv
%   - Results_Plots/Step_Response_Graphs/reference_trajectories_overview.png

clear; clc; close all;

%% 1. Set paths
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fullfile(scriptDir, '..', '..');
addpath(fullfile(projectRoot, 'MATLAB_Simulink', 'common_functions'));

rawDir   = fullfile(projectRoot, 'Data', 'Raw_SimData');
plotsDir = fullfile(projectRoot, 'Results_Plots', 'Step_Response_Graphs');
if ~exist(rawDir, 'dir'), mkdir(rawDir); end
if ~exist(plotsDir, 'dir'), mkdir(plotsDir); end

%% 2. Load Plant Parameters
p = boost_converter_params();
dt = p.dt;       % 1e-5 s (10 us)
Tsim = 0.2;      % 0.2 s (200 ms extended duration per trajectory)
N = round(Tsim / dt);
time_s = (0:N-1)' * dt;

% Derivative filter parameters: s / (tau_f*s + 1)
tau_f = p.tau_f; % 0.002 s
alpha_f = tau_f / (tau_f + dt);
beta_f  = 1.0 / (tau_f + dt);

% Reference supervisor controller gains (tuned to match plant dynamics without windup)
Kp = 0.002;
Ki = 0.03;
Kd = 0.00001;

%% 4. Define 24 Diverse Operational Scenarios
scenarios = {
    % ID, Name, Vin_profile_fn, Vref_profile_fn, R_profile_fn
    1,  'Nominal Startup',              @(t) 20,                          @(t) 40,                         @(t) 500;
    2,  'Low Input Startup (15V)',       @(t) 15,                          @(t) 40,                         @(t) 500;
    3,  'High Input Startup (25V)',      @(t) 25,                          @(t) 40,                         @(t) 500;
    4,  'Heavy Load Startup (250 Ohm)',  @(t) 20,                          @(t) 40,                         @(t) 250;
    5,  'Light Load Startup (750 Ohm)',  @(t) 20,                          @(t) 40,                         @(t) 750;
    6,  'Low Setpoint Startup (35V)',    @(t) 20,                          @(t) 35,                         @(t) 500;
    7,  'High Setpoint Startup (45V)',   @(t) 20,                          @(t) 45,                         @(t) 500;
    8,  'Vref Step Up (40V -> 45V)',     @(t) 20,                          @(t) 40 + 5*(t>=0.05),           @(t) 500;
    9,  'Vref Step Down (40V -> 35V)',   @(t) 20,                          @(t) 40 - 5*(t>=0.05),           @(t) 500;
    10, 'Vin Step Up (20V -> 25V)',      @(t) 20 + 5*(t>=0.05),            @(t) 40,                         @(t) 500;
    11, 'Vin Step Down (20V -> 15V)',    @(t) 20 - 5*(t>=0.05),            @(t) 40,                         @(t) 500;
    12, 'Vin Step Up Minor (20V->22V)',  @(t) 20 + 2*(t>=0.05),            @(t) 40,                         @(t) 500;
    13, 'Vin Step Down Minor (20V->18V)',@(t) 20 - 2*(t>=0.05),           @(t) 40,                         @(t) 500;
    14, 'Vin Swing Low-to-High (15->25)',@(t) 15 + 10*(t>=0.05),          @(t) 40,                         @(t) 500;
    15, 'Vin Swing High-to-Low (25->15)',@(t) 25 - 10*(t>=0.05),          @(t) 40,                         @(t) 500;
    16, 'Load Step Drop (500 -> 350)',   @(t) 20,                          @(t) 40,                         @(t) 500 - 150*(t>=0.05);
    17, 'Load Step Rise (500 -> 700)',   @(t) 20,                          @(t) 40,                         @(t) 500 + 200*(t>=0.05);
    18, 'Heavy Load Step (500 -> 250)',  @(t) 20,                          @(t) 40,                         @(t) 500 - 250*(t>=0.05);
    19, 'Combined Dip (Vin 18V, R 400)', @(t) 20 - 2*(t>=0.05),            @(t) 40,                         @(t) 500 - 100*(t>=0.05);
    20, 'Combined Surge(Vin 23V, R 600)',@(t) 20 + 3*(t>=0.05),           @(t) 40,                         @(t) 500 + 100*(t>=0.05);
    21, 'Ramp Vref Tracking (40->44V)',  @(t) 20,                          @(t) 40 + 200*max(0,min(0.02,t-0.05)), @(t) 500;
    22, 'High Vin Light Load (24V, 650)',@(t) 24,                          @(t) 40,                         @(t) 650;
    23, 'Low Vin Heavy Load (16V, 350)', @(t) 16,                          @(t) 40,                         @(t) 350;
    24, 'Multi-step Vin (20->23->17V)',  @(t) 20 + 3*(t>=0.04) - 6*(t>=0.10), @(t) 40,                  @(t) 500;
};

numScenarios = size(scenarios, 1);
fprintf('=== Generating Trajectory Dataset Across %d Scenarios on Same Plant ===\n', numScenarios);

%% 5. Simulate Trajectories
all_run_id        = [];
all_time_s        = [];
all_Vin_V         = [];
all_Vref_V        = [];
all_Vout_V        = [];
all_iL_A          = [];
all_error_V       = [];
all_err_deriv_Vps = [];
all_duty_target   = [];

trajectoryLogs = struct();

for s = 1:numScenarios
    runId    = scenarios{s, 1};
    scenName = scenarios{s, 2};
    vin_fn   = scenarios{s, 3};
    vref_fn  = scenarios{s, 4};
    rload_fn = scenarios{s, 5};

    % Unenergized startup
    x = [0; 0];
    int_err  = 0;
    prev_err = 0;
    prev_der = 0;

    t_arr    = zeros(N, 1);
    vin_arr  = zeros(N, 1);
    vref_arr = zeros(N, 1);
    vout_arr = zeros(N, 1);
    il_arr   = zeros(N, 1);
    err_arr  = zeros(N, 1);
    eder_arr = zeros(N, 1);
    duty_arr = zeros(N, 1);

    for k = 1:N
        t = time_s(k);
        Vin_k  = vin_fn(t);
        Vref_k = vref_fn(t);
        R_k    = rload_fn(t);

        Vout_k = x(2);
        iL_k   = x(1);

        e = Vref_k - Vout_k;
        if k == 1
            de_dt = 0;
        else
            de_dt = alpha_f * prev_der + beta_f * (e - prev_err);
        end

        int_err = int_err + e * dt;

        % Feedforward nominal duty cycle: D_ff = 1 - Vin / Vref
        d_ff = max(p.d_min, min(p.d_max, 1 - Vin_k / max(Vref_k, 1)));

        % Proportional-Integral-Derivative control command
        d_raw = d_ff + Kp * e + Ki * int_err + Kd * de_dt;

        % Symmetrical anti-windup clamping to [d_min, d_max]
        d_sat = min(max(d_raw, p.d_min), p.d_max);
        if (d_raw >= p.d_max && e > 0) || (d_raw <= p.d_min && e < 0)
            int_err = int_err - e * dt;
        end

        % Exact unconstrained averaged CCM boost converter ODE
        dx = boost_converter_ode(x, d_sat, Vin_k, p, R_k);
        x  = x + dt * dx;

        % State physical limits
        x(1) = max(0, x(1));
        x(2) = max(0, x(2));

        t_arr(k)    = t;
        vin_arr(k)  = Vin_k;
        vref_arr(k) = Vref_k;
        vout_arr(k) = Vout_k;
        il_arr(k)   = iL_k;
        err_arr(k)  = e;
        eder_arr(k) = de_dt;
        duty_arr(k) = d_sat;

        prev_err = e;
        prev_der = de_dt;
    end

    trajectoryLogs(s).run_id          = runId;
    trajectoryLogs(s).name            = scenName;
    trajectoryLogs(s).time_s          = t_arr;
    trajectoryLogs(s).Vin_V           = vin_arr;
    trajectoryLogs(s).Vref_V          = vref_arr;
    trajectoryLogs(s).Vout_V          = vout_arr;
    trajectoryLogs(s).iL_A            = il_arr;
    trajectoryLogs(s).error_V         = err_arr;
    trajectoryLogs(s).error_deriv_Vps = eder_arr;
    trajectoryLogs(s).duty_target     = duty_arr;

    all_run_id        = [all_run_id; repmat(runId, N, 1)]; %#ok<AGROW>
    all_time_s        = [all_time_s; t_arr]; %#ok<AGROW>
    all_Vin_V         = [all_Vin_V; vin_arr]; %#ok<AGROW>
    all_Vref_V        = [all_Vref_V; vref_arr]; %#ok<AGROW>
    all_Vout_V        = [all_Vout_V; vout_arr]; %#ok<AGROW>
    all_iL_A          = [all_iL_A; il_arr]; %#ok<AGROW>
    all_error_V       = [all_error_V; err_arr]; %#ok<AGROW>
    all_err_deriv_Vps = [all_err_deriv_Vps; eder_arr]; %#ok<AGROW>
    all_duty_target   = [all_duty_target; duty_arr]; %#ok<AGROW>

    fprintf('  Run %02d/%02d: %-34s -> Vout_end = %.2f V, D_end = %.4f\n', ...
        runId, numScenarios, scenName, vout_arr(end), duty_arr(end));
end

%% 6. Compile Raw Dataset Table
rawTable = table(all_run_id, all_time_s, all_Vin_V, all_Vref_V, all_Vout_V, ...
    all_iL_A, all_error_V, all_err_deriv_Vps, all_duty_target, ...
    'VariableNames', {'run_id', 'time_s', 'Vin_V', 'Vref_V', 'Vout_V', ...
                      'iL_A', 'error_V', 'error_deriv_Vps', 'duty_target'});

%% 7. Subsample to 10 kHz Controller Frequency
subsampleRatio = 10;
subsampledIdx = 1:subsampleRatio:height(rawTable);
rawTable_subsampled = rawTable(subsampledIdx, :);

fprintf('\nTotal generated samples: %d -> Subsampled (10 kHz): %d\n', ...
    height(rawTable), height(rawTable_subsampled));

%% 8. Save Files
matFile = fullfile(rawDir, 'anfis_reference_trajectories.mat');
csvFile = fullfile(rawDir, 'anfis_reference_trajectories.csv');

save(matFile, 'rawTable', 'rawTable_subsampled', 'trajectoryLogs', 'scenarios', 'p');
fprintf('Saved MAT file: %s\n', matFile);

writetable(rawTable_subsampled, csvFile);
fprintf('Saved CSV file: %s\n', csvFile);

%% 9. Overview Plots
fig = figure('Visible', 'off', 'Position', [100 100 1200 800]);

subplot(3, 1, 1); hold on; grid on;
for s = [1, 8, 10, 11, 16, 18, 20]
    plot(trajectoryLogs(s).time_s * 1000, trajectoryLogs(s).Vout_V, 'LineWidth', 1.3, ...
        'DisplayName', sprintf('Run %d: %s', s, trajectoryLogs(s).name));
end
plot(trajectoryLogs(1).time_s * 1000, trajectoryLogs(1).Vref_V, '--k', 'LineWidth', 1.6, 'DisplayName', 'Reference (40V)');
ylabel('V_{out} (V)', 'FontSize', 10);
title('Averaged Boost Plant: Output Voltage Trajectories Across Key Scenarios', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 8);

subplot(3, 1, 2); hold on; grid on;
for s = [1, 8, 10, 11, 16, 18, 20]
    plot(trajectoryLogs(s).time_s * 1000, trajectoryLogs(s).duty_target, 'LineWidth', 1.3, ...
        'DisplayName', sprintf('Run %d: %s', s, trajectoryLogs(s).name));
end
ylabel('Duty Target D', 'FontSize', 10);
title('Reference Commanded Duty Cycle Target D \in [0.05, 0.95]', 'FontSize', 11, 'FontWeight', 'bold');
ylim([0 1]); legend('Location', 'southeast', 'FontSize', 8);

subplot(3, 1, 3); hold on; grid on;
for s = [1, 8, 10, 11, 16, 18, 20]
    plot(trajectoryLogs(s).time_s * 1000, trajectoryLogs(s).iL_A, 'LineWidth', 1.3, ...
        'DisplayName', sprintf('Run %d: %s', s, trajectoryLogs(s).name));
end
ylabel('i_L (A)', 'FontSize', 10); xlabel('Time (ms)', 'FontSize', 10);
title('Inductor Current Trajectories (Natural Peak, No 5A Clamp)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 8);

plotPath = fullfile(plotsDir, 'reference_trajectories_overview.png');
saveas(fig, plotPath);
close(fig);
fprintf('Saved overview plot: %s\n', plotPath);
fprintf('=== Data Generation from Same Plant Complete ===\n');
