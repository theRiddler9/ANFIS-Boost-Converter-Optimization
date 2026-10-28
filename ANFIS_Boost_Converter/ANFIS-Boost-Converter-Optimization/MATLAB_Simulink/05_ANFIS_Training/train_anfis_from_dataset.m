%% TRAIN_ANFIS_FROM_DATASET
% Trains a Sugeno-type Adaptive Neuro-Fuzzy Inference System (ANFIS) on the
% validated boost converter dataset using hybrid learning (Least-Squares + Backpropagation).
%
% Workflow:
%   1. Loads train, validation, and test datasets.
%   2. Constructs initial FIS via Grid Partitioning (5x5 generalized bell MFs).
%   3. Trains ANFIS with validation monitoring to prevent overfitting.
%   4. Evaluates held-out independent test set.
%   5. Saves trained FIS to MATLAB_Simulink/06_ANFIS_Deployment/anfis_trained.fis
%   6. Saves training and test metrics to Data/Results/
%   7. Generates convergence and prediction accuracy plots in Results_Plots/
%
% Inputs to ANFIS:
%   Input 1: error_V (Voltage error in Volts: Vref - Vout)
%   Input 2: error_diff_V (Discrete change in error: e(k) - e(k-1) in Volts)
% Target:
%   duty_target (Commanded duty cycle D in [0.05, 0.95])

clear; clc; close all;

%% 1. File Paths
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fullfile(scriptDir, '..', '..');

trainDataDir = fullfile(projectRoot, 'Data', 'Training_Datasets');
resultsDir   = fullfile(projectRoot, 'Data', 'Results');
plotsDir     = fullfile(projectRoot, 'Results_Plots', 'PID_vs_Fuzzy_vs_ANFIS');
deployDir1   = fullfile(projectRoot, 'MATLAB_Simulink', '06_ANFIS_Deployment');
deployDir2   = fullfile(projectRoot, 'MATLAB_Simulink', '6_ANFIS_Deployment');

if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end
if ~exist(plotsDir, 'dir'), mkdir(plotsDir); end
if ~exist(deployDir1, 'dir'), mkdir(deployDir1); end
if ~exist(deployDir2, 'dir'), mkdir(deployDir2); end

%% 2. Load Datasets
trainData = readtable(fullfile(trainDataDir, 'anfis_training_data.csv'));
valData   = readtable(fullfile(trainDataDir, 'anfis_validation_data.csv'));
testData  = readtable(fullfile(trainDataDir, 'anfis_test_data.csv'));

fprintf('=== Training ANFIS on Validated Boost Converter Dataset ===\n');
fprintf('  Train samples:      %d\n', height(trainData));
fprintf('  Validation samples: %d\n', height(valData));
fprintf('  Test samples:       %d\n', height(testData));

% Prepare matrices [Input1, Input2, Target]
X_train = [trainData.error_V, trainData.error_diff_V];
Y_train = trainData.duty_target;

X_val = [valData.error_V, valData.error_diff_V];
Y_val = valData.duty_target;

X_test = [testData.error_V, testData.error_diff_V];
Y_test = testData.duty_target;

trainMatrix = [X_train, Y_train];
valMatrix   = [X_val, Y_val];
testMatrix  = [X_test, Y_test];

%% 3. Generate Initial FIS Structure via Grid Partitioning
fprintf('\nGenerating Initial Sugeno FIS structure (Grid Partitioning: 5x5 MFs)...\n');
genOpt = genfisOptions('GridPartition');
genOpt.NumMembershipFunctions = [5, 5];
genOpt.InputMembershipFunctionType = 'gbellmf';

initialFis = genfis(X_train, Y_train, genOpt);
initialFis.Inputs(1).Name = 'error_V';
initialFis.Inputs(2).Name = 'error_diff_V';
initialFis.Outputs(1).Name = 'duty_target';

%% 4. Configure ANFIS Training Options
numEpochs = 40;
anfisOpt = anfisOptions( ...
    'InitialFIS', initialFis, ...
    'EpochNumber', numEpochs, ...
    'ValidationData', valMatrix, ...
    'DisplayANFISInformation', 0, ...
    'DisplayErrorValues', 1, ...
    'DisplayStepSize', 0, ...
    'DisplayFinalResults', 1);

%% 5. Train ANFIS with Validation Monitoring
fprintf('Executing ANFIS Hybrid Training over %d epochs...\n', numEpochs);
[trainedFis, trainError, stepSize, chkFis, chkError] = anfis(trainMatrix, anfisOpt);

%% 6. Select Best Model (Using Checking FIS if validation improved)
bestFis = trainedFis;
if ~isempty(chkFis)
    fprintf('Selecting model with minimal validation error from checkpointing.\n');
    bestFis = chkFis;
end

%% 7. Evaluate Performance Across Splits
pred_train = evalfis(bestFis, X_train);
pred_val   = evalfis(bestFis, X_val);
pred_test  = evalfis(bestFis, X_test);

% Bound predictions to valid physical duty limits
pred_train = min(max(pred_train, 0.05), 0.95);
pred_val   = min(max(pred_val, 0.05), 0.95);
pred_test  = min(max(pred_test, 0.05), 0.95);

% Metrics computation
train_rmse = sqrt(mean((pred_train - Y_train).^2));
train_mae  = mean(abs(pred_train - Y_train));
train_maxe = max(abs(pred_train - Y_train));

val_rmse   = sqrt(mean((pred_val - Y_val).^2));
val_mae    = mean(abs(pred_val - Y_val));
val_maxe   = max(abs(pred_val - Y_val));

test_rmse  = sqrt(mean((pred_test - Y_test).^2));
test_mae   = mean(abs(pred_test - Y_test));
test_maxe  = max(abs(pred_test - Y_test));

fprintf('\n=================================================================\n');
fprintf('                 ANFIS EVALUATION METRICS                        \n');
fprintf('=================================================================\n');
fprintf('Split       | RMSE       | MAE        | Max Absolute Error\n');
fprintf('-----------------------------------------------------------------\n');
fprintf('Train       | %10.5f | %10.5f | %10.5f\n', train_rmse, train_mae, train_maxe);
fprintf('Validation  | %10.5f | %10.5f | %10.5f\n', val_rmse, val_mae, val_maxe);
fprintf('Test (Held) | %10.5f | %10.5f | %10.5f\n', test_rmse, test_mae, test_maxe);
fprintf('=================================================================\n');

%% 8. Export Metrics to CSV
trainMetricsTable = table({'Train'; 'Validation'}, [train_rmse; val_rmse], [train_mae; val_mae], [train_maxe; val_maxe], ...
    'VariableNames', {'Split', 'RMSE', 'MAE', 'MaxError'});
writetable(trainMetricsTable, fullfile(resultsDir, 'anfis_training_metrics.csv'));

testMetricsTable = table({'HeldOutTest'}, test_rmse, test_mae, test_maxe, ...
    'VariableNames', {'Split', 'RMSE', 'MAE', 'MaxError'});
writetable(testMetricsTable, fullfile(resultsDir, 'anfis_test_metrics.csv'));
fprintf('Saved metrics CSVs to: %s\n', resultsDir);

%% 9. Save Trained FIS Files
fisPath1 = fullfile(deployDir1, 'anfis_trained.fis');
fisPath2 = fullfile(deployDir2, 'anfis_trained.fis');
writeFIS(bestFis, fisPath1);
writeFIS(bestFis, fisPath2);
fprintf('Saved trained FIS to: %s\n', fisPath1);

%% 10. Generate Deliverable Plots
% Plot A: Training & Validation Loss Convergence
figA = figure('Visible', 'off', 'Position', [100 100 800 500]);
plot(trainError, 'LineWidth', 1.8, 'Color', [0.1 0.4 0.8], 'DisplayName', 'Training Error (RMSE)');
hold on; grid on;
if ~isempty(chkError)
    plot(chkError, 'LineWidth', 1.8, 'Color', [0.85 0.3 0.1], 'DisplayName', 'Validation Error (RMSE)');
end
xlabel('Epoch', 'FontSize', 11);
ylabel('Root Mean Squared Error (Duty Command)', 'FontSize', 11);
title('ANFIS Training & Validation Convergence Curve', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 10);
plotPathA = fullfile(plotsDir, 'anfis_loss_convergence.png');
saveas(figA, plotPathA);
close(figA);
fprintf('Saved plot: %s\n', plotPathA);

% Plot B: Predicted vs Target Duty on Test Set
figB = figure('Visible', 'off', 'Position', [100 100 800 600]);
scatter(Y_test, pred_test, 12, [0.2 0.5 0.8], 'filled', 'MarkerFaceAlpha', 0.6);
hold on; grid on;
plot([0 1], [0 1], '--r', 'LineWidth', 1.8, 'DisplayName', 'Ideal Match (Y = T)');
xlim([0 1]); ylim([0 1]);
xlabel('Target Duty Cycle (Reference Controller)', 'FontSize', 11);
ylabel('Predicted Duty Cycle (ANFIS)', 'FontSize', 11);
title(sprintf('Held-Out Test Set: ANFIS Predicted vs Target Duty (RMSE = %.4f)', test_rmse), ...
    'FontSize', 12, 'FontWeight', 'bold');
legend('Predictions', 'Ideal Line', 'Location', 'southeast', 'FontSize', 10);
plotPathB = fullfile(plotsDir, 'anfis_pred_vs_target_test.png');
saveas(figB, plotPathB);
close(figB);
fprintf('Saved plot: %s\n', plotPathB);

% Plot C: Test Residual Error Histogram
figC = figure('Visible', 'off', 'Position', [100 100 800 500]);
residuals = pred_test - Y_test;
histogram(residuals, 40, 'FaceColor', [0.3 0.7 0.4], 'EdgeColor', 'none');
grid on;
xlabel('Duty Cycle Residual Error (Predicted - Target)', 'FontSize', 11);
ylabel('Sample Count', 'FontSize', 11);
title(sprintf('Held-Out Test Set: Prediction Residuals (MAE = %.4f, MaxErr = %.4f)', test_mae, test_maxe), ...
    'FontSize', 12, 'FontWeight', 'bold');
plotPathC = fullfile(plotsDir, 'anfis_residual_error_test.png');
saveas(figC, plotPathC);
close(figC);
fprintf('Saved plot: %s\n', plotPathC);

fprintf('=== ANFIS Training and Evaluation Complete ===\n');
