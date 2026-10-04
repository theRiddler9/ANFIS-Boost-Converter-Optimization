%% BUILD_FUZZY_FIS
% Builds the classic 7x7 = 49-rule Mamdani fuzzy controller (error,
% change-of-error -> duty cycle) replicating the base paper's manually
% designed controller. Saves it as fuzzy_49rules.fis so it can be loaded
% by evalfis() in simulate_closed_loop.m, with no Simulink model required
% to test it standalone.
%
% Run this once to generate the .fis file, then use it in
% 07_Comparative_Testing/run_all_controllers.m

clear; clc;

%% 1) Create the FIS object
fis = mamfis('Name', 'BoostConverterFuzzy49');

%% 2) Input 1: error = Vref - Vout (normalized range, tune to your system)
fis = addInput(fis, [-50 50], 'Name', 'error');
labels = {'NB','NM','NS','Z','PS','PM','PB'};
centers = linspace(-50, 50, 7);
for i = 1:7
    fis = addMF(fis, 'error', 'trimf', mfRange(centers, i, -50, 50), 'Name', labels{i});
end

%% 3) Input 2: change-of-error (derivative of error)
fis = addInput(fis, [-500 500], 'Name', 'd_error');
centers2 = linspace(-500, 500, 7);
for i = 1:7
    fis = addMF(fis, 'd_error', 'trimf', mfRange(centers2, i, -500, 500), 'Name', labels{i});
end

%% 4) Output: duty cycle correction
fis = addOutput(fis, [0 1], 'Name', 'duty');
centersO = linspace(0, 1, 7);
for i = 1:7
    fis = addMF(fis, 'duty', 'trimf', mfRange(centersO, i, 0, 1), 'Name', labels{i});
end

%% 5) Standard 7x7 diagonal rule table (classic fuzzy-PI rule surface)
% Rows = error (NB..PB), Cols = d_error (NB..PB)
% Output index into labels (1=NB ... 7=PB)
ruleTable = [
    1 1 1 2 3 4 5
    1 1 2 3 4 5 6
    1 2 3 4 5 6 7
    2 3 4 4 4 5 6
    3 4 5 6 7 6 7
    4 5 6 7 7 7 7
    5 6 7 7 7 7 7
];

ruleList = [];
for i = 1:7      % error MF index
    for j = 1:7  % d_error MF index
        out = ruleTable(i, j);
        ruleList = [ruleList; i j out 1 1]; %#ok<AGROW>
    end
end
fis = addRule(fis, ruleList);

%% 6) Save
outPath = fullfile(fileparts(mfilename('fullpath')), 'fuzzy_49rules.fis');
writeFIS(fis, outPath);
fprintf('Saved 49-rule fuzzy controller to: %s\n', outPath);

%% 7) Quick sanity plot
figure;
gensurf(fis);
title('Fuzzy Controller Rule Surface (error, d\_error -> duty)');

%% Local helper function
function mf = mfRange(centers, idx, lo, hi)
    % Builds a triangular MF around centers(idx), clamped to [lo hi],
    % with endpoints flattened into shoulders for first/last MF.
    step = centers(2) - centers(1);
    a = centers(idx) - step;
    b = centers(idx);
    c = centers(idx) + step;
    if idx == 1, a = lo - step; end
    if idx == numel(centers), c = hi + step; end
    mf = [a b c];
end
