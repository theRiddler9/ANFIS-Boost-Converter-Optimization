%% TRAIN_ANFIS_FROM_DATASET
% Train a 25-rule Sugeno ANFIS on the same-plant dataset.
% Inputs are normalized identically to the manual fuzzy controller.
% The checking-set minimum is the deployed early-stopping checkpoint.

clear; clc; close all;
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fullfile(scriptDir, '..', '..');
addpath(fullfile(projectRoot, 'MATLAB_Simulink', 'common_functions'));
trainDataDir = fullfile(projectRoot, 'Data', 'Training_Datasets');
resultsDir = fullfile(projectRoot, 'Data', 'Results');
plotsDir = fullfile(projectRoot, 'Results_Plots', 'PID_vs_Fuzzy_vs_ANFIS');
deployDirs = {fullfile(projectRoot, 'MATLAB_Simulink', '06_ANFIS_Deployment'), ...
              fullfile(projectRoot, 'MATLAB_Simulink', '6_ANFIS_Deployment')};
for k = 1:numel(deployDirs), if ~exist(deployDirs{k}, 'dir'), mkdir(deployDirs{k}); end, end
if ~exist(resultsDir, 'dir'), mkdir(resultsDir); end
if ~exist(plotsDir, 'dir'), mkdir(plotsDir); end

tr = readtable(fullfile(trainDataDir, 'anfis_training_data.csv'));
va = readtable(fullfile(trainDataDir, 'anfis_validation_data.csv'));
te = readtable(fullfile(trainDataDir, 'anfis_test_data.csv'));
p = boost_converter_params();
Xtr = [tr.error_V / p.error_scale, tr.error_deriv_Vps / p.derivative_scale];
Ytr = tr.duty_target;
Xva = [va.error_V / p.error_scale, va.error_deriv_Vps / p.derivative_scale];
Yva = va.duty_target;
Xte = [te.error_V / p.error_scale, te.error_deriv_Vps / p.derivative_scale];
% Deployment clamps both controller inputs to the common normalized range.
% Apply the same operation before FIS generation so its learned membership
% ranges and parameters are valid for every deployed input.
Xtr = min(max(Xtr, -1), 1);
Xva = min(max(Xva, -1), 1);
Xte = min(max(Xte, -1), 1);
Yte = te.duty_target;
assert(all(isfinite([Xtr(:); Ytr; Xva(:); Yva; Xte(:); Yte])));
rng(42, 'twister');
fprintf('Training ANFIS: train=%d validation=%d test=%d samples\n', height(tr), height(va), height(te));


genOpt = genfisOptions('GridPartition');
genOpt.NumMembershipFunctions = [5 5];
genOpt.InputMembershipFunctionType = 'gbellmf';
fis0 = genfis(Xtr, Ytr, genOpt);
fis0.Inputs(1).Name = 'error_norm';
fis0.Inputs(2).Name = 'derivative_norm';
fis0.Inputs(1).Range = [-1 1];
fis0.Inputs(2).Range = [-1 1];
fis0.Outputs(1).Name = 'duty_target';

opts = anfisOptions('InitialFIS', fis0, 'EpochNumber', 40, ...
    'ValidationData', [Xva Yva], 'DisplayANFISInformation', 0, ...
    'DisplayErrorValues', 1, 'DisplayStepSize', 0, 'DisplayFinalResults', 0);
[trainedFis, trainErr, ~, checkFis, checkErr] = anfis([Xtr Ytr], opts);
if isempty(checkErr)
    checkErr = sqrt(mean((evalfis(trainedFis, Xva) - Yva).^2));
    checkFis = trainedFis;
end
if iscell(checkFis), bestFis = checkFis{1}; else, bestFis = checkFis; end
bestFis.Inputs(1).Range = [-1 1];
bestFis.Inputs(2).Range = [-1 1];
[bestVal, bestEpoch] = min(checkErr(:));
if isempty(bestEpoch) || ~isfinite(bestVal)
    bestFis = trainedFis;
    bestEpoch = numel(trainErr);
    bestVal = sqrt(mean((evalfis(bestFis, Xva) - Yva).^2));
end

predTrain = min(max(evalfis(bestFis, Xtr), p.d_min), p.d_max);
predVal = min(max(evalfis(bestFis, Xva), p.d_min), p.d_max);
predTest = min(max(evalfis(bestFis, Xte), p.d_min), p.d_max);
trainRMSE = sqrt(mean((predTrain - Ytr).^2));
valRMSE = sqrt(mean((predVal - Yva).^2));
testRMSE = sqrt(mean((predTest - Yte).^2));
testMAE = mean(abs(predTest - Yte));
testMaxE = max(abs(predTest - Yte));
fprintf('Best validation epoch=%d, val RMSE=%.6f, test RMSE=%.6f, MAE=%.6f, MaxErr=%.6f\n', ...
    bestEpoch, valRMSE, testRMSE, testMAE, testMaxE);

for k = 1:numel(deployDirs), writeFIS(bestFis, fullfile(deployDirs{k}, 'anfis_trained.fis')); end
anfisModel.params1 = zeros(5, 3);
anfisModel.params2 = zeros(5, 3);
anfisModel.consequents = zeros(25, 3);
for k = 1:5
    anfisModel.params1(k,:) = bestFis.Inputs(1).MembershipFunctions(k).Parameters;
    anfisModel.params2(k,:) = bestFis.Inputs(2).MembershipFunctions(k).Parameters;
end
for k = 1:25
    anfisModel.consequents(k,:) = bestFis.Outputs(1).MembershipFunctions(k).Parameters;
end
save(fullfile(resultsDir, 'anfis_model.mat'), 'anfisModel', 'bestEpoch', ...
    'trainRMSE', 'valRMSE', 'testRMSE', 'testMAE', 'testMaxE', 'trainErr', 'checkErr');

json = struct('best_epoch', bestEpoch, 'train_rmse', trainRMSE, ...
    'val_rmse', valRMSE, 'test_rmse', testRMSE, 'test_mae', testMAE, ...
    'test_maxe', testMaxE, 'params1', anfisModel.params1, ...
    'params2', anfisModel.params2, 'consequents', anfisModel.consequents);
fid = fopen(fullfile(resultsDir, 'anfis_model.json'), 'w');
fwrite(fid, jsonencode(json, 'PrettyPrint', true)); fclose(fid);

writetable(table({'Train'; 'Validation'; 'HeldOutTest'}, ...
    [trainRMSE; valRMSE; testRMSE], [NaN; NaN; testMAE], ...
    [NaN; NaN; testMaxE], [bestEpoch; bestEpoch; bestEpoch], ...
    'VariableNames', {'Split','RMSE','MAE','MaxError','OptimalEpoch'}), ...
    fullfile(resultsDir, 'anfis_training_metrics.csv'));

fig = figure('Visible', 'off');
plot(trainErr, 'LineWidth', 1.5); hold on; plot(checkErr, 'LineWidth', 1.5);
xline(bestEpoch, '--k', sprintf('Best epoch %d', bestEpoch)); grid on;
xlabel('Epoch'); ylabel('RMSE'); title('ANFIS training and validation convergence');
legend('Training','Validation','Location','best');
saveas(fig, fullfile(plotsDir, 'anfis_loss_convergence.png')); close(fig);
fprintf('Saved early-stopped model and metrics to %s\n', resultsDir);
