%% GENERATE_TRAINING_DATA
% Step 4 of the methodology: hook up an aggressive, near-ideal PID
% controller (the "perfect teacher") to the boost converter and run it
% through disturbance tests, logging [error, d_error, duty_cycle] at
% every timestep. This becomes the ANFIS training dataset.
%
% This script uses the pure-MATLAB averaged model (common_functions/),
% so you can run it RIGHT NOW even before your Simulink .slx models
% exist. Once 01_Boost_Converter_Model/boost_converter_openloop.slx is
% built, you can swap simulate_closed_loop() for a sim() call on the
% model with minimal changes -- the training data format stays the same.

clear; clc; close all;
addpath(fullfile('..', 'common_functions'));

p = boost_converter_params();

%% 1) Aggressive "ideal teacher" PID gains
% These are intentionally much more aggressive than the PID baseline
% gains you'll use in 02_PID_Controller -- this controller only exists
% to generate near-optimal reference data, not to be a candidate itself.
ideal.Kp = 0.02;
ideal.Ki = 5;
ideal.Kd = 0.0005;

%% 2) Define a sweep of input-voltage disturbance scenarios
% Each scenario: input voltage steps from Vin_start to Vin_end at t_step
scenarios = [
    20 15 0.02;
    20 25 0.02;
    20 18 0.02;
    20 22 0.02;
    15 25 0.02;
    25 15 0.02;
];

all_err   = [];
all_derr  = [];
all_duty  = [];

for s = 1:size(scenarios, 1)
    Vin_start = scenarios(s, 1);
    Vin_end   = scenarios(s, 2);
    t_step    = scenarios(s, 3);

    Vin_profile = @(t) Vin_start + (Vin_end - Vin_start) * (t >= t_step);

    log = simulate_closed_loop('PID', ideal, Vin_profile, p);

    d_error = [0, diff(log.err)] / p.dt;

    all_err  = [all_err,  log.err]; %#ok<AGROW>
    all_derr = [all_derr, d_error]; %#ok<AGROW>
    all_duty = [all_duty, log.d];   %#ok<AGROW>

    fprintf('Scenario %d/%d done: %dV -> %dV\n', s, size(scenarios,1), Vin_start, Vin_end);
end

%% 3) Build training table [error, d_error -> duty]
trainingData = [all_err(:), all_derr(:), all_duty(:)];

% Optional: subsample if the dataset is too large for ANFIS training
% (full 1us-step logs can be huge). Keep every Nth point.
subsampleEvery = 20;
trainingData = trainingData(1:subsampleEvery:end, :);

fprintf('Training dataset size: %d rows\n', size(trainingData, 1));

%% 4) Save
outPath = fullfile('..', '..', 'Data', 'Training_Datasets', 'anfis_training_data.mat');
save(outPath, 'trainingData');
fprintf('Saved training data to: %s\n', outPath);

%% 5) Quick visual sanity check
figure;
subplot(3,1,1); plot(trainingData(:,1)); ylabel('error'); title('Training Data Preview');
subplot(3,1,2); plot(trainingData(:,2)); ylabel('d\_error');
subplot(3,1,3); plot(trainingData(:,3)); ylabel('duty'); xlabel('sample #');
