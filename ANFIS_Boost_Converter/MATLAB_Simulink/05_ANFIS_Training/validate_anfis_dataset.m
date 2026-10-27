%% VALIDATE_ANFIS_DATASET
% Performs rigorous physical and numerical validation checks on the prepared
% ANFIS training, validation, and test datasets.
%
% Checks Performed:
%   1. File existence and accessibility
%   2. Non-empty tabular structure and column presence
%   3. Absence of NaN, Inf, or empty values
%   4. Target duty cycle bounds [d_min, d_max] = [0.05, 0.95]
%   5. Complete split independence (no shared run_ids between Train, Val, Test)
%   6. Physical plausibility checks (Vout >= 0, iL >= 0, Vin within [14, 26] V)
%   7. Statistical distributions and summary metrics
%
% Outputs:
%   - Data/Results/anfis_dataset_validation_report.txt
%   - Results_Plots/Comparison_Tables/anfis_dataset_distributions.png

clear; clc; close all;

%% 1. Paths
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fullfile(scriptDir, '..', '..');

trainDataDir = fullfile(projectRoot, 'Data', 'Training_Datasets');
resultsDir   = fullfile(projectRoot, 'Data', 'Results');
plotsDir     = fullfile(projectRoot, 'Results_Plots', 'Comparison_Tables');

if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end
if ~exist(plotsDir, 'dir'), mkdir(plotsDir); end

trainCsv = fullfile(trainDataDir, 'anfis_training_data.csv');
valCsv   = fullfile(trainDataDir, 'anfis_validation_data.csv');
testCsv  = fullfile(trainDataDir, 'anfis_test_data.csv');

reportPath = fullfile(resultsDir, 'anfis_dataset_validation_report.txt');
repFid = fopen(reportPath, 'w');

fprintf('=================================================================\n');
fprintf('                ANFIS DATASET VALIDATION SUITE                   \n');
fprintf('=================================================================\n\n');

fprintf(repFid, '=================================================================\n');
fprintf(repFid, '                ANFIS DATASET VALIDATION SUITE                   \n');
fprintf(repFid, '=================================================================\n\n');
fprintf(repFid, 'Validation Date: %s\n\n', char(datetime('now')));

passCount = 0;
failCount = 0;

%% Check 1: File Existence
filesExist = exist(trainCsv, 'file') && exist(valCsv, 'file') && exist(testCsv, 'file');
if filesExist
    logCheck('Check 1: File existence of Train, Val, Test CSVs', true, repFid);
    passCount = passCount + 1;
else
    logCheck('Check 1: File existence of Train, Val, Test CSVs', false, repFid);
    failCount = failCount + 1;
    fclose(repFid);
    error('Dataset files missing!');
end

trainData = readtable(trainCsv);
valData   = readtable(valCsv);
testData  = readtable(testCsv);

%% Check 2: Dimension and Non-Empty Checks
reqCols = {'run_id', 'time_s', 'Vin_V', 'Vref_V', 'Vout_V', 'iL_A', 'error_V', 'error_diff_V', 'error_deriv_Vps', 'duty_target'};
hasCols = all(ismember(reqCols, trainData.Properties.VariableNames)) && ...
          all(ismember(reqCols, valData.Properties.VariableNames)) && ...
          all(ismember(reqCols, testData.Properties.VariableNames));
nonEmpty = (height(trainData) > 0) && (height(valData) > 0) && (height(testData) > 0);

if hasCols && nonEmpty
    logCheck(sprintf('Check 2: Column schema & dimensions (Train: %d, Val: %d, Test: %d)', ...
        height(trainData), height(valData), height(testData)), true, repFid);
    passCount = passCount + 1;
else
    logCheck('Check 2: Column schema & dimensions', false, repFid);
    failCount = failCount + 1;
end

%% Check 3: Non-Finite Values (NaN, Inf)
allData = [trainData; valData; testData];
hasNanInf = false;
for i = 1:length(reqCols)
    c = allData.(reqCols{i});
    if any(isnan(c)) || any(isinf(c))
        hasNanInf = true;
        break;
    end
end
if ~hasNanInf
    logCheck('Check 3: Absence of NaN and Inf values across all columns', true, repFid);
    passCount = passCount + 1;
else
    logCheck('Check 3: Absence of NaN and Inf values across all columns', false, repFid);
    failCount = failCount + 1;
end

%% Check 4: Target Duty Cycle Bounds [0.05, 0.95]
minDuty = min(allData.duty_target);
maxDuty = max(allData.duty_target);
dutyBounded = (minDuty >= 0.05) && (maxDuty <= 0.95);
if dutyBounded
    logCheck(sprintf('Check 4: Duty cycle limits [0.05, 0.95] (Observed range: [%.4f, %.4f])', ...
        minDuty, maxDuty), true, repFid);
    passCount = passCount + 1;
else
    logCheck(sprintf('Check 4: Duty cycle limits violated (Observed: [%.4f, %.4f])', ...
        minDuty, maxDuty), false, repFid);
    failCount = failCount + 1;
end

%% Check 5: Split Independence (No Shared Run IDs)
trainRuns = unique(trainData.run_id);
valRuns   = unique(valData.run_id);
testRuns  = unique(testData.run_id);

int_tv = intersect(trainRuns, valRuns);
int_tt = intersect(trainRuns, testRuns);
int_vt = intersect(valRuns, testRuns);
independent = isempty(int_tv) && isempty(int_tt) && isempty(int_vt);

if independent
    logCheck(sprintf('Check 5: Split independence (Train runs: %d, Val runs: %d, Test runs: %d, 0 overlap)', ...
        length(trainRuns), length(valRuns), length(testRuns)), true, repFid);
    passCount = passCount + 1;
else
    logCheck('Check 5: Split independence (Overlap detected!)', false, repFid);
    failCount = failCount + 1;
end

%% Check 6: Physical Plausibility Checks
voutMin = min(allData.Vout_V);
ilMin   = min(allData.iL_A);
vinMin  = min(allData.Vin_V);
vinMax  = max(allData.Vin_V);
physicsOk = (voutMin >= 0) && (ilMin >= 0) && (vinMin >= 14) && (vinMax <= 26);

if physicsOk
    logCheck(sprintf('Check 6: Physical plausibility (Vout_min=%.2fV, iL_min=%.3fA, Vin in [%.1f, %.1f]V)', ...
        voutMin, ilMin, vinMin, vinMax), true, repFid);
    passCount = passCount + 1;
else
    logCheck('Check 6: Physical plausibility violated', false, repFid);
    failCount = failCount + 1;
end

%% Summary Statistics Table
fprintf('\n--- Variable Statistics Across Entire Dataset ---\n');
fprintf(repFid, '\n--- Variable Statistics Across Entire Dataset ---\n');
varList = {'Vin_V', 'Vref_V', 'Vout_V', 'iL_A', 'error_V', 'error_diff_V', 'error_deriv_Vps', 'duty_target'};
fprintf('%-18s | %-10s | %-10s | %-10s | %-10s\n', 'Variable', 'Min', 'Max', 'Mean', 'StdDev');
fprintf(repFid, '%-18s | %-10s | %-10s | %-10s | %-10s\n', 'Variable', 'Min', 'Max', 'Mean', 'StdDev');
fprintf('%s\n', repmat('-', 1, 65));
fprintf(repFid, '%s\n', repmat('-', 1, 65));

for v = 1:length(varList)
    vn = varList{v};
    col = allData.(vn);
    fprintf('%-18s | %10.4f | %10.4f | %10.4f | %10.4f\n', vn, min(col), max(col), mean(col), std(col));
    fprintf(repFid, '%-18s | %10.4f | %10.4f | %10.4f | %10.4f\n', vn, min(col), max(col), mean(col), std(col));
end

%% Final Verdict
fprintf('\n=================================================================\n');
fprintf('VALIDATION SUMMARY: %d PASSED, %d FAILED\n', passCount, failCount);
fprintf('=================================================================\n');

fprintf(repFid, '\n=================================================================\n');
fprintf(repFid, 'VALIDATION SUMMARY: %d PASSED, %d FAILED\n', passCount, failCount);
fprintf(repFid, 'OVERALL STATUS: %s\n', ternary(failCount == 0, 'PASS', 'FAIL'));
fprintf(repFid, '=================================================================\n');
fclose(repFid);
fprintf('Validation report written to: %s\n', reportPath);

%% Generate Distribution Plot
fig = figure('Visible', 'off', 'Position', [100 100 1100 700]);

subplot(2, 2, 1);
histogram(allData.error_V, 40, 'FaceColor', [0.2 0.4 0.8], 'EdgeColor', 'none');
title('Distribution: Voltage Error e (V)'); xlabel('Error (V)'); ylabel('Count'); grid on;

subplot(2, 2, 2);
histogram(allData.error_deriv_Vps, 40, 'FaceColor', [0.8 0.4 0.2], 'EdgeColor', 'none');
title('Distribution: Error Derivative de/dt (V/s)'); xlabel('de/dt (V/s)'); ylabel('Count'); grid on;

subplot(2, 2, 3);
histogram(allData.duty_target, 40, 'FaceColor', [0.2 0.7 0.3], 'EdgeColor', 'none');
title('Distribution: Target Duty Command D'); xlabel('Duty Target'); ylabel('Count'); grid on;

subplot(2, 2, 4);
scatter(allData.error_V(1:5:end), allData.duty_target(1:5:end), 8, allData.Vin_V(1:5:end), 'filled');
colormap('parula'); colorbar;
title('State-Action Mapping: Error vs Duty Command');
xlabel('Voltage Error (V)'); ylabel('Duty Target D'); grid on;

plotOut = fullfile(plotsDir, 'anfis_dataset_distributions.png');
saveas(fig, plotOut);
close(fig);
fprintf('Saved distribution plot to: %s\n', plotOut);

%% Helper Function
function logCheck(desc, passed, fid)
    if passed
        status = 'PASS';
    else
        status = 'FAIL';
    end
    fprintf('[%s] %s\n', status, desc);
    fprintf(fid, '[%s] %s\n', status, desc);
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
