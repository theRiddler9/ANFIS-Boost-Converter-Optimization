%% GENERATE_ANFIS_TRAINING_DATA
% Generates a physically validated, supervised dataset for training an
% Adaptive Neuro-Fuzzy Inference System (ANFIS) controller for the DC-DC
% boost converter.
%
% Plant Model:
%   Averaged CCM DC-DC Boost Converter:
%     L * diL/dt = Vin - (1-D)*Vout - rL*iL
%     C * dVout/dt = (1-D)*iL - Vout/R
%   Nominal parameters: Vin=20V, Vref=40V, L=500uH, C=1000uF, R=500 Ohm, rL=0.1 Ohm
%
% Reference Controller:
%   Tuned discrete-time anti-windup PID controller providing stable, high-performance
%   duty-cycle commands D in [0.05, 0.95].
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

rawDir = fullfile(projectRoot, 'Data', 'Raw_SimData');
plotsDir = fullfile(projectRoot, 'Results_Plots', 'Step_Response_Graphs');
if ~exist(rawDir, 'dir'), mkdir(rawDir); end
if ~exist(plotsDir, 'dir'), mkdir(plotsDir); end

%% 2. Load Plant Parameters
p = boost_converter_params();
dt = p.dt; % 1e-5 s (10 us)
Tsim = p.Tsim; % 0.05 s
N = round(Tsim / dt);
time_s = (0:N-1)' * dt;

%% 3. Reference Controller Gains (Tuned with Anti-Windup)
% Feedforward base nominal duty D0 = 1 - Vin/Vref = 0.5
% Proportional, Integral, Derivative gains tuned for fast settling without excessive overshoot
Kp = 0.006;
Ki = 0.8;
Kd = 0.00004;

%% 4. Define 24 Diverse Operating Scenarios
% Covering startup transients, setpoint changes, line disturbances, and load steps
scenarios = {
    % ID, Name, Vin_profile_fn, Vref_profile_fn, R_profile_fn
    1,  'Nominal Startup',             @(t) 20,                          @(t) 40,                         @(t) 500;
    2,  'Low Input Startup (15V)',      @(t) 15,                          @(t) 40,                         @(t) 500;
    3,  'High Input Startup (25V)',     @(t) 25,                          @(t) 40,                         @(t) 500;
    4,  'Heavy Load Startup (250 Ohm)', @(t) 20,                          @(t) 40,                         @(t) 250;
    5,  'Light Load Startup (750 Ohm)', @(t) 20,                          @(t) 40,                         @(t) 750;
    6,  'Low Setpoint Startup (35V)',   @(t) 20,                          @(t) 35,                         @(t) 500;
    7,  'High Setpoint Startup (45V)',  @(t) 20,                          @(t) 45,                         @(t) 500;
    8,  'Vref Step Up (40V -> 45V)',    @(t) 20,                          @(t) 40 + 5*(t>=0.025),          @(t) 500;
    9,  'Vref Step Down (40V -> 35V)',  @(t) 20,                          @(t) 40 - 5*(t>=0.025),          @(t) 500;
    10, 'Vin Step Up (20V -> 25V)',     @(t) 20 + 5*(t>=0.025),           @(t) 40,                         @(t) 500;
    11, 'Vin Step Down (20V -> 15V)',   @(t) 20 - 5*(t>=0.025),           @(t) 40,                         @(t) 500;
    12, 'Vin Step Up Minor (20V->22V)', @(t) 20 + 2*(t>=0.025),           @(t) 40,                         @(t) 500;
    13, 'Vin Step Down Minor (20V->18V)',@(t) 20 - 2*(t>=0.025),          @(t) 40,                         @(t) 500;
    14, 'Vin Swing Low-to-High (15->25)',@(t) 15 + 10*(t>=0.025),         @(t) 40,                         @(t) 500;
    15, 'Vin Swing High-to-Low (25->15)',@(t) 25 - 10*(t>=0.025),         @(t) 40,                         @(t) 500;
    16, 'Load Step Drop (500 -> 350)',  @(t) 20,                          @(t) 40,                         @(t) 500 - 150*(t>=0.025);
    17, 'Load Step Rise (500 -> 700)',  @(t) 20,                          @(t) 40,                         @(t) 500 + 200*(t>=0.025);
    18, 'Heavy Load Step (500 -> 250)', @(t) 20,                          @(t) 40,                         @(t) 500 - 250*(t>=0.025);
    19, 'Combined Dip (Vin 18V, R 400)',@(t) 20 - 2*(t>=0.025),           @(t) 40,                         @(t) 500 - 100*(t>=0.025);
    20, 'Combined Surge(Vin 23V, R 600)',@(t) 20 + 3*(t>=0.025),          @(t) 40,                         @(t) 500 + 100*(t>=0.025);
    21, 'Ramp Vref Tracking (40->44V)', @(t) 20,                          @(t) 40 + 400*max(0,min(0.01,t-0.02)), @(t) 500;
    22, 'High Vin Low Load Steady',     @(t) 24,                          @(t) 40,                         @(t) 650;
    23, 'Low Vin High Load Steady',     @(t) 16,                          @(t) 40,                         @(t) 350;
    24, 'Multi-step Vin (20->22->17V)', @(t) 20 + 2*(t>=0.015) - 5*(t>=0.035), @(t) 40,                 @(t) 500;
};

numScenarios = size(scenarios, 1);
fprintf('=== Generating Trajectory Dataset Across %d Scenarios ===\n', numScenarios);

%% 5. Simulate Trajectories
all_run_id      = [];
all_time_s      = [];
all_Vin_V       = [];
all_Vref_V      = [];
all_Vout_V      = [];
all_iL_A        = [];
all_error_V     = [];
all_err_diff_V  = [];
all_err_deriv_Vps = [];
all_duty_target = [];

trajectoryLogs = struct();

for s = 1:numScenarios
    runId       = scenarios{s, 1};
    scenName    = scenarios{s, 2};
    vin_fn      = scenarios{s, 3};
    vref_fn     = scenarios{s, 4};
    rload_fn    = scenarios{s, 5};
    
    % Initial states: x = [iL; vC] starting at 0 (unenergized startup)
    x = [0; 0];
    int_err = 0;
    prev_err = 0;
    
    t_arr     = zeros(N, 1);
    vin_arr   = zeros(N, 1);
    vref_arr  = zeros(N, 1);
    vout_arr  = zeros(N, 1);
    il_arr    = zeros(N, 1);
    err_arr   = zeros(N, 1);
    ediff_arr = zeros(N, 1);
    eder_arr  = zeros(N, 1);
    duty_arr  = zeros(N, 1);
    
    for k = 1:N
        t = time_s(k);
        Vin_k  = vin_fn(t);
        Vref_k = vref_fn(t);
        R_k    = rload_fn(t);
        
        Vout_k = x(2);
        iL_k   = x(1);
        
        e = Vref_k - Vout_k;
        de_dt = (e - prev_err) / dt;
        de_diff = e - prev_err;
        
        % Integrator update
        int_err = int_err + e * dt;
        
        % Feedforward nominal duty estimation based on current operating point
        d_ff = max(0, 1 - (Vin_k / max(Vref_k, 1)));
        
        % PID control law
        d_raw = d_ff + Kp * e + Ki * int_err + Kd * de_dt;
        
        % Anti-windup clamping to [d_min, d_max]
        d_sat = min(max(d_raw, p.d_min), p.d_max);
        if d_sat ~= d_raw
            % Freeze integrator to prevent windup
            int_err = int_err - e * dt;
        end
        
        % Step plant forward via averaged CCM ODE
        dx = boost_converter_ode(x, d_sat, Vin_k, p, R_k);
        x  = x + dt * dx;
        
        % Physical boundary protection (no negative output or reverse diode current)
        x(1) = max(0, x(1));
        x(2) = max(0, x(2));
        
        % Record timestep
        t_arr(k)     = t;
        vin_arr(k)   = Vin_k;
        vref_arr(k)  = Vref_k;
        vout_arr(k)  = Vout_k;
        il_arr(k)    = iL_k;
        err_arr(k)   = e;
        ediff_arr(k) = de_diff;
        eder_arr(k)  = de_dt;
        duty_arr(k)  = d_sat;
        
        prev_err = e;
    end
    
    % Store trajectory log
    trajectoryLogs(s).run_id = runId;
    trajectoryLogs(s).name = scenName;
    trajectoryLogs(s).time_s = t_arr;
    trajectoryLogs(s).Vin_V = vin_arr;
    trajectoryLogs(s).Vref_V = vref_arr;
    trajectoryLogs(s).Vout_V = vout_arr;
    trajectoryLogs(s).iL_A = il_arr;
    trajectoryLogs(s).error_V = err_arr;
    trajectoryLogs(s).error_diff_V = ediff_arr;
    trajectoryLogs(s).error_deriv_Vps = eder_arr;
    trajectoryLogs(s).duty_target = duty_arr;
    
    % Append to pooled dataset
    all_run_id      = [all_run_id; repmat(runId, N, 1)]; %#ok<AGROW>
    all_time_s      = [all_time_s; t_arr]; %#ok<AGROW>
    all_Vin_V       = [all_Vin_V; vin_arr]; %#ok<AGROW>
    all_Vref_V      = [all_Vref_V; vref_arr]; %#ok<AGROW>
    all_Vout_V      = [all_Vout_V; vout_arr]; %#ok<AGROW>
    all_iL_A        = [all_iL_A; il_arr]; %#ok<AGROW>
    all_error_V     = [all_error_V; err_arr]; %#ok<AGROW>
    all_err_diff_V  = [all_err_diff_V; ediff_arr]; %#ok<AGROW>
    all_err_deriv_Vps = [all_err_deriv_Vps; eder_arr]; %#ok<AGROW>
    all_duty_target = [all_duty_target; duty_arr]; %#ok<AGROW>
    
    fprintf('  Run %02d/%02d completed: %-32s (Vout_end = %.2f V, D_end = %.4f)\n', ...
        runId, numScenarios, scenName, vout_arr(end), duty_arr(end));
end

%% 6. Compile Trajectory Table
rawTable = table(all_run_id, all_time_s, all_Vin_V, all_Vref_V, all_Vout_V, ...
    all_iL_A, all_error_V, all_err_diff_V, all_err_deriv_Vps, all_duty_target, ...
    'VariableNames', {'run_id', 'time_s', 'Vin_V', 'Vref_V', 'Vout_V', ...
                      'iL_A', 'error_V', 'error_diff_V', 'error_deriv_Vps', 'duty_target'});

%% 7. Subsample to Practical Controller Frequency (10 kHz, 100 us)
% High-speed Euler is dt=10us (100 kHz). Subsampling by factor of 10 gives
% realistic 10 kHz controller samples (500 samples/run * 24 runs = 12,000 samples)
subsampleRatio = 10;
subsampledIdx = 1:subsampleRatio:height(rawTable);
rawTable_subsampled = rawTable(subsampledIdx, :);

fprintf('Raw trajectory samples: %d -> Subsampled (10 kHz): %d\n', ...
    height(rawTable), height(rawTable_subsampled));

%% 8. Save Raw Outputs
matFile = fullfile(rawDir, 'anfis_reference_trajectories.mat');
csvFile = fullfile(rawDir, 'anfis_reference_trajectories.csv');

save(matFile, 'rawTable', 'rawTable_subsampled', 'trajectoryLogs', 'scenarios', 'p');
fprintf('Saved MAT file: %s\n', matFile);

writetable(rawTable_subsampled, csvFile);
fprintf('Saved CSV file: %s\n', csvFile);

%% 9. Generate Overview Plots
fig = figure('Visible', 'off', 'Position', [100 100 1200 800]);

subplot(3, 1, 1); hold on; grid on;
for s = [1, 8, 10, 11, 16, 21]
    plot(trajectoryLogs(s).time_s * 1000, trajectoryLogs(s).Vout_V, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('Run %d: %s', s, trajectoryLogs(s).name));
end
plot(trajectoryLogs(1).time_s * 1000, trajectoryLogs(1).Vref_V, '--k', 'LineWidth', 1.5, 'DisplayName', 'Vref Nominal (40V)');
ylabel('V_{out} (V)'); title('Reference Converter Output Voltage Across Key Scenarios');
legend('Location', 'southeast', 'FontSize', 8);

subplot(3, 1, 2); hold on; grid on;
for s = [1, 8, 10, 11, 16, 21]
    plot(trajectoryLogs(s).time_s * 1000, trajectoryLogs(s).duty_target, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('Run %d: %s', s, trajectoryLogs(s).name));
end
ylabel('Target Duty D'); title('Reference Controller Commanded Duty Cycle Target');
ylim([0 1]); legend('Location', 'southeast', 'FontSize', 8);

subplot(3, 1, 3); hold on; grid on;
for s = [1, 8, 10, 11, 16, 21]
    plot(trajectoryLogs(s).time_s * 1000, trajectoryLogs(s).iL_A, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('Run %d: %s', s, trajectoryLogs(s).name));
end
ylabel('i_L (A)'); xlabel('Time (ms)'); title('Inductor Current Trajectories');
legend('Location', 'southeast', 'FontSize', 8);

plotPath = fullfile(plotsDir, 'reference_trajectories_overview.png');
saveas(fig, plotPath);
close(fig);
fprintf('Saved plot to: %s\n', plotPath);
fprintf('=== Data Generation Complete ===\n');
