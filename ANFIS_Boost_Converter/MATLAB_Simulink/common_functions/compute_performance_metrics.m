function m = compute_performance_metrics(log, p)
%COMPUTE_PERFORMANCE_METRICS Computes comprehensive transient and steady-state
%performance metrics from numerical time-series logs.
%
% Inputs:
%   log : struct with fields: t, vC, iL, d, err
%   p   : parameter struct (must contain Vref)
%
% Output m struct with fields:
%   - IAE             : Integral of Absolute Error (V*s)
%   - ISE             : Integral of Squared Error (V^2*s)
%   - ITAE            : Integral of Time-multiplied Absolute Error (V*s^2)
%   - ResponseTime_ms : Time to first reach within 2% of Vref (ms)
%   - Overshoot_pct   : Peak overshoot percentage above Vref (%)
%   - SettlingTime_ms : Time after which Vout stays strictly within 2% of Vref (ms)
%   - SteadyStateErr_V: Mean absolute error over the final 10% of simulation window (V)
%   - Peak_iL_A       : Maximum inductor current (A)
%   - Duty_min        : Minimum commanded duty ratio
%   - Duty_max        : Maximum commanded duty ratio

t    = log.t;
vC   = log.vC;
iL   = log.iL;
d    = log.d;
e    = log.err;
Vref = p.Vref;
dt   = t(2) - t(1);

% Integral performance indices
m.IAE  = sum(abs(e)) * dt;
m.ISE  = sum(e.^2) * dt;
m.ITAE = sum(t .* abs(e)) * dt;

% Overshoot (%)
m.Overshoot_pct = max(0, (max(vC) - Vref) / Vref * 100);

% 2% tolerance band: [0.98*Vref, 1.02*Vref]
tol = 0.02 * Vref;
withinBand = abs(vC - Vref) <= tol;

% Response time (ms): time to first reach within 2% of reference
firstIdx = find(withinBand, 1, 'first');
if isempty(firstIdx)
    m.ResponseTime_ms = NaN;
else
    m.ResponseTime_ms = t(firstIdx) * 1000;
end

% Settling time (ms): time after which signal STAYS inside 2% band
outsideIdx = find(~withinBand, 1, 'last');
if isempty(outsideIdx)
    m.SettlingTime_ms = 0;
elseif outsideIdx == numel(t)
    m.SettlingTime_ms = t(end) * 1000; % Did not settle within window
else
    m.SettlingTime_ms = t(outsideIdx + 1) * 1000;
end

% Steady-state error (V): mean absolute error in final 10% tail
tailIdx = round(0.9 * numel(vC)):numel(vC);
m.SteadyStateErr_V = mean(abs(Vref - vC(tailIdx)));

% Current and duty metrics
m.Peak_iL_A = max(iL);
m.Duty_min  = min(d);
m.Duty_max  = max(d);

end
