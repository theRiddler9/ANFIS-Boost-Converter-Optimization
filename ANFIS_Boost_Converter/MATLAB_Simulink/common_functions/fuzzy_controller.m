function d = fuzzy_controller(e, de, p)
%FUZZY_CONTROLLER Hand-coded 7-term zero-order Sugeno fuzzy controller
% Replicating Model 3 (boost_converter_fuzzy) and Fig. 7 of the paper.
%
% Inputs:
%   e  : Voltage error (Vref - Vout) in Volts
%   de : Filtered error derivative in V/s
%   p  : Converter parameters struct (boost_converter_params)
%
% Output:
%   d  : Commanded duty cycle, strictly saturated to [p.d_min, p.d_max]

if nargin < 3 || isempty(p)
    d_min = 0.05;
    d_max = 0.95;
else
    d_min = p.d_min;
    d_max = p.d_max;
end

% Common normalized inputs shared with ANFIS.
errorScale = 40;
derivativeScale = 200;
if nargin >= 3 && isfield(p, 'error_scale'), errorScale = p.error_scale; end
if nargin >= 3 && isfield(p, 'derivative_scale'), derivativeScale = p.derivative_scale; end
en = min(max(e / errorScale, -1), 1);
dn = min(max(de / derivativeScale, -1), 1);

% Seven triangular membership-function centers: NB, NM, NS, Z, PS, PM, PB
c = [-1, -2/3, -1/3, 0, 1/3, 2/3, 1];
muE = zeros(1, 7);
muD = zeros(1, 7);

for k = 1:7
    muE(k) = max(0, 1 - abs(en - c(k)) / (1/3));
    muD(k) = max(0, 1 - abs(dn - c(k)) / (1/3));
end

% Shoulder membership sets at boundaries
if en <= -1, muE(1) = 1; end
if en >= 1,  muE(7) = 1; end
if dn <= -1, muD(1) = 1; end
if dn >= 1,  muD(7) = 1; end

% Seven Sugeno singletons: duty corrections around nominal 0.5
out = [-0.25, -0.17, -0.0833, 0, 0.0833, 0.17, 0.25];

num = 0;
den = 0;

for i = 1:7
    for j = 1:7
        % Error raises duty; derivative provides damping
        idx = round(4 + (i - 4) + 0.5 * (j - 4));
        idx = min(max(idx, 1), 7);
        w = muE(i) * muD(j);
        num = num + w * out(idx);
        den = den + w;
    end
end

if den > 1e-9
    correction = num / den;
else
    correction = 0;
end

d_raw = 0.5 + correction;
d = min(max(d_raw, d_min), d_max);

end
