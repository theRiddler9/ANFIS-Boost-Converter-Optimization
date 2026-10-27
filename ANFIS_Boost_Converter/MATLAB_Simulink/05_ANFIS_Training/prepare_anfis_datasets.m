%% PREPARE_ANFIS_DATASETS
% Partitions scenarios by complete simulation runs (run_id) into
% Train (16 runs, ~67%), Validation (4 runs, ~17%), and Test (4 runs, ~17%)
% sets to ensure ZERO temporal data leakage. Exports standardized CSVs,
% summary metrics, JSON metadata, and feature distribution plots.

clear; clc; close all;

%% 1. File Paths
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fullfile(scriptDir, '..', '..');

rawMatPath   = fullfile(projectRoot, 'Data', 'Raw_SimData', 'anfis_reference_trajectories.mat');
trainDataDir = fullfile(projectRoot, 'Data', 'Training_Datasets');
plotsDir     = fullfile(projectRoot, 'Results_Plots', 'Comparison_Tables');

if ~exist(trainDataDir, 'dir'), mkdir(trainDataDir); end
if ~exist(plotsDir, 'dir'), mkdir(plotsDir); end

if ~exist(rawMatPath, 'file')
    error('Raw trajectories MAT file not found at: %s. Run generate_anfis_training_data.m first.', rawMatPath);
end

%% 2. Load Raw Trajectory Data
loaded = load(rawMatPath);
data = loaded.rawTable_subsampled;
scenarios = loaded.scenarios;
numRuns = size(scenarios, 1);

fprintf('=== Preparing ANFIS Datasets from %d Trajectories (%d Subsampled Rows) ===\n', ...
    numRuns, height(data));

%% 3. Stratified Partitioning by Whole Trajectory Run ID
% Fixed random seed for reproducibility
rng(42, 'twister');

% Partition scheme ensuring diverse operating regimes in all sets:
train_runs = [1, 2, 4, 6, 7, 8, 10, 11, 12, 13, 16, 17, 19, 21, 22, 23]; % 16 runs (~67%)
val_runs   = [3, 9, 14, 20];                                             % 4 runs (~17%)
test_runs  = [5, 15, 18, 24];                                            % 4 runs (~17%)

% Verify partition exclusivity and completeness
all_assigned = [train_runs, val_runs, test_runs];
assert(length(unique(all_assigned)) == numRuns, 'Partition must be mutually exclusive and exhaustive!');
assert(isempty(intersect(train_runs, val_runs)), 'Leakage between train and val!');
assert(isempty(intersect(train_runs, test_runs)), 'Leakage between train and test!');
assert(isempty(intersect(val_runs, test_runs)), 'Leakage between val and test!');

%% 4. Extract Datasets
trainData = data(ismember(data.run_id, train_runs), :);
valData   = data(ismember(data.run_id, val_runs), :);
testData  = data(ismember(data.run_id, test_runs), :);

fprintf('Dataset Splits:\n');
fprintf('  Train Set:      %5d samples (%2d runs: %s)\n', height(trainData), length(train_runs), mat2str(train_runs));
fprintf('  Validation Set: %5d samples (%2d runs: %s)\n', height(valData), length(val_runs), mat2str(val_runs));
fprintf('  Test Set:       %5d samples (%2d runs: %s)\n', height(testData), length(test_runs), mat2str(test_runs));

%% 5. Export Datasets to CSV
trainCsv = fullfile(trainDataDir, 'anfis_training_data.csv');
valCsv   = fullfile(trainDataDir, 'anfis_validation_data.csv');
testCsv  = fullfile(trainDataDir, 'anfis_test_data.csv');

writetable(trainData, trainCsv);
writetable(valData, valCsv);
writetable(testData, testCsv);

fprintf('Saved: %s\n', trainCsv);
fprintf('Saved: %s\n', valCsv);
fprintf('Saved: %s\n', testCsv);

%% 6. Generate Summary Table (anfis_dataset_summary.csv)
vars = {'Vin_V', 'Vref_V', 'Vout_V', 'iL_A', 'error_V', 'error_deriv_Vps', 'duty_target'};
splits = {'Train', 'Validation', 'Test'};
splitData = {trainData, valData, testData};

summaryTable = table();
for sp = 1:3
    sName = splits{sp};
    sData = splitData{sp};
    for v = 1:length(vars)
        vName = vars{v};
        col = sData.(vName);
        newRow = table(string(sName), string(vName), min(col), max(col), mean(col), std(col), ...
            'VariableNames', {'Split', 'Variable', 'Min', 'Max', 'Mean', 'StdDev'});
        summaryTable = [summaryTable; newRow]; %#ok<AGROW>
    end
end

summaryCsv = fullfile(trainDataDir, 'anfis_dataset_summary.csv');
writetable(summaryTable, summaryCsv);
fprintf('Saved: %s\n', summaryCsv);

%% 7. Export Metadata JSON (anfis_dataset_metadata.json)
metadata = struct();
metadata.projectName = 'ANFIS-Controlled DC-DC Boost Converter';
metadata.modelType = 'Averaged Continuous-Conduction-Mode (CCM)';
metadata.creationDate = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
metadata.generatorScript = 'generate_anfis_training_data.m';
metadata.preparerScript = 'prepare_anfis_datasets.m';
metadata.randomSeed = 42;
metadata.sampleTime_s = loaded.p.dt * 10; % 100 us (10 kHz)
metadata.totalRuns = numRuns;
metadata.totalSamples = height(data);

metadata.plantParameters = struct(...
    'Vin_nom_V', loaded.p.Vin_nom, ...
    'Vref_nom_V', loaded.p.Vref, ...
    'L_H', loaded.p.L, ...
    'C_F', loaded.p.C, ...
    'R_Ohm', loaded.p.R, ...
    'rL_Ohm', loaded.p.rL, ...
    'fsw_Hz', loaded.p.fsw, ...
    'duty_min', loaded.p.d_min, ...
    'duty_max', loaded.p.d_max, ...
    'tau_f_s', loaded.p.tau_f);

metadata.splits = struct(...
    'train', struct('numRuns', length(train_runs), 'runIds', train_runs, 'samples', height(trainData)), ...
    'validation', struct('numRuns', length(val_runs), 'runIds', val_runs, 'samples', height(valData)), ...
    'test', struct('numRuns', length(test_runs), 'runIds', test_runs, 'samples', height(testData)));

metadata.anfisSignals = struct(...
    'input1', struct('name', 'error_V', 'description', 'Voltage error (Vref - Vout) in Volts'), ...
    'input2', struct('name', 'error_deriv_Vps', 'description', 'Filtered error derivative (de/dt) in V/s'), ...
    'target', struct('name', 'duty_target', 'description', 'Commanded duty cycle D in [0.05, 0.95]'));

jsonStr = jsonencode(metadata, 'PrettyPrint', true);
metaJsonPath = fullfile(trainDataDir, 'anfis_dataset_metadata.json');
fid = fopen(metaJsonPath, 'w');
fwrite(fid, jsonStr);
fclose(fid);
fprintf('Saved: %s\n', metaJsonPath);

%% 8. Feature Distributions Plot
fig = figure('Visible', 'off', 'Position', [100 100 1200 600]);

subplot(2, 2, 1);
histogram(trainData.error_V, 35, 'FaceColor', [0.2 0.5 0.8], 'EdgeColor', 'none');
grid on; xlabel('Error V_{ref} - V_{out} (V)'); ylabel('Count');
title('Distribution: Voltage Error e(t)');

subplot(2, 2, 2);
histogram(trainData.error_deriv_Vps, 35, 'FaceColor', [0.8 0.4 0.2], 'EdgeColor', 'none');
grid on; xlabel('Filtered Error Derivative de/dt (V/s)'); ylabel('Count');
title('Distribution: Error Derivative \Delta e / \Delta t');

subplot(2, 2, 3);
histogram(trainData.duty_target, 35, 'FaceColor', [0.3 0.7 0.4], 'EdgeColor', 'none');
grid on; xlabel('Target Duty Cycle D'); ylabel('Count');
title('Distribution: Commanded Duty Cycle D \in [0.05, 0.95]');

subplot(2, 2, 4);
scatter(trainData.error_V, trainData.duty_target, 10, trainData.Vin_V, 'filled');
colorbar; colormap(parula); grid on;
xlabel('Error (V)'); ylabel('Duty Cycle D');
title('State-Action Map: Error vs Duty (Colored by Vin)');

distPlotPath = fullfile(plotsDir, 'anfis_dataset_distributions.png');
saveas(fig, distPlotPath);
close(fig);
fprintf('Saved distribution plot: %s\n', distPlotPath);
fprintf('=== Dataset Preparation Complete ===\n');
