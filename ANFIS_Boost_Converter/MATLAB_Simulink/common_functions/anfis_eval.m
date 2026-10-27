function d = anfis_eval(e, de, anfisModel, p)
%ANFIS_EVAL Evaluates a 5x5 (25 rule) first-order Sugeno ANFIS controller.
% Works standalone without requiring Fuzzy Logic Toolbox.
%
% Inputs:
%   e          : Voltage error (Vref - Vout) in Volts
%   de         : Filtered error derivative (de/dt) in V/s
%   anfisModel : struct with fields:
%                  .params1     : [5 x 3] generalized bell [a, b, c] for input 1
%                  .params2     : [5 x 3] generalized bell [a, b, c] for input 2
%                  .consequents : [25 x 3] linear coefficients [p, q, r]
%                OR loaded FIS struct / path to .fis file
%   p          : Parameter struct with p.d_min and p.d_max
%
% Output:
%   d          : Commanded duty cycle strictly limited to [p.d_min, p.d_max]

if nargin < 4 || isempty(p)
    d_min = 0.05;
    d_max = 0.95;
else
    d_min = p.d_min;
    d_max = p.d_max;
end

% Apply the common controller scaling before evaluating the learned model.
errorScale = 40;
derivativeScale = 200;
if nargin >= 4 && isfield(p, 'error_scale'), errorScale = p.error_scale; end
if nargin >= 4 && isfield(p, 'derivative_scale'), derivativeScale = p.derivative_scale; end
en = min(max(e / errorScale, -1), 1);
den = min(max(de / derivativeScale, -1), 1);

% FIS objects are evaluated directly using the common scaled inputs.
if isobject(anfisModel)
    d = evalfis(anfisModel, [en den]);
    d = min(max(double(d), d_min), d_max);
    return;
end

% If anfisModel is a char / path, load it
if ischar(anfisModel) || isstring(anfisModel)
    anfisModel = load_fis_or_mat(anfisModel);
end

% Extract parameters
if isfield(anfisModel, 'params1') && isfield(anfisModel, 'params2') && isfield(anfisModel, 'consequents')
    params1 = anfisModel.params1;
    params2 = anfisModel.params2;
    conseq  = anfisModel.consequents;
else
    error('anfisModel must contain fields: params1, params2, consequents');
end

% Layer 1: Generalized Bell Membership Functions: 1 / (1 + |(x - c)/a|^(2b))
mu1 = zeros(1, 5);
for i = 1:5
    a = params1(i, 1);
    b = params1(i, 2);
    c = params1(i, 3);
    denom = 1 + abs((en - c) / max(a, 1e-6))^(2 * b);
    mu1(i) = 1 / max(denom, 1e-12);
end

mu2 = zeros(1, 5);
for j = 1:5
    a = params2(j, 1);
    b = params2(j, 2);
    c = params2(j, 3);
    denom = 1 + abs((den - c) / max(a, 1e-6))^(2 * b);
    mu2(j) = 1 / max(denom, 1e-12);
end

% Layer 2: Firing strengths (product AND)
w = zeros(1, 25);
r = 1;
for i = 1:5
    for j = 1:5
        w(r) = mu1(i) * mu2(j);
        r = r + 1;
    end
end

% Layer 3: Normalized firing strengths
sumW = sum(w);
if sumW > 1e-12
    w_norm = w / sumW;
else
    w_norm = ones(1, 25) / 25;
end

% Layer 4 & 5: First-order Sugeno evaluation: sum(w_norm * (p*e + q*de + r))
d_raw = 0;
for r = 1:25
    p_coef = conseq(r, 1);
    q_coef = conseq(r, 2);
    r_const = conseq(r, 3);
    f_r = p_coef * en + q_coef * den + r_const;
    d_raw = d_raw + w_norm(r) * f_r;
end

% Symmetrical duty cycle saturation to identical physical bounds
d = min(max(d_raw, d_min), d_max);

end

function model = load_fis_or_mat(filePath)
    if endsWith(filePath, '.mat')
        loaded = load(filePath);
        if isfield(loaded, 'model')
            model = loaded.model;
        else
            model = loaded;
        end
    elseif endsWith(filePath, '.fis')
        model = readfis(filePath);
    else
        error('Unsupported file format: %s', filePath);
    end
end

function model = parse_fis_file(fisPath)
    fid = fopen(fisPath, 'r');
    if fid == -1
        error('Cannot open FIS file: %s', fisPath);
    end
    
    params1 = zeros(5, 3);
    params2 = zeros(5, 3);
    consequents = zeros(25, 3);
    
    currentInput = 0;
    
    while ~feof(fid)
        line = strtrim(fgetl(fid));
        if startsWith(line, '[Input1]')
            currentInput = 1;
        elseif startsWith(line, '[Input2]')
            currentInput = 2;
        elseif startsWith(line, '[Output1]')
            currentInput = 3;
        elseif startsWith(line, 'MF') && contains(line, 'gbellmf')
            % Parse gbellmf: [a b c]
            tok = regexp(line, 'gbellmf'',\s*\[([^\]]+)\]', 'tokens');
            if ~isempty(tok)
                vals = str2num(tok{1}{1}); %#ok<ST2NM>
                mfIdx = sscanf(line, 'MF%d');
                if currentInput == 1 && mfIdx <= 5
                    params1(mfIdx, :) = vals;
                elseif currentInput == 2 && mfIdx <= 5
                    params2(mfIdx, :) = vals;
                end
            end
        elseif startsWith(line, 'MF') && contains(line, 'linear')
            % Parse linear: [p q r]
            tok = regexp(line, 'linear'',\s*\[([^\]]+)\]', 'tokens');
            if ~isempty(tok)
                vals = str2num(tok{1}{1}); %#ok<ST2NM>
                mfIdx = sscanf(line, 'MF%d');
                if mfIdx <= 25
                    consequents(mfIdx, :) = vals;
                end
            end
        end
    end
    fclose(fid);
    
    model.params1 = params1;
    model.params2 = params2;
    model.consequents = consequents;
end
