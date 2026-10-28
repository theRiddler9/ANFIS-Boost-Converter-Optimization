function [d, int_err_new] = pid_controller(err, int_err, prev_err, Kp, Ki, Kd, dt, p)
%PID_CONTROLLER Discrete PID duty-cycle controller.
%
%   [d, int_err_new] = pid_controller(err, int_err, prev_err, Kp, Ki, Kd, dt, p)
%
% err      : current error (Vref - Vout)
% int_err  : accumulated integral of error so far
% prev_err : error at the previous timestep (for derivative term)
% Kp, Ki, Kd : PID gains (tuned for the unified plant)
% dt       : simulation/control timestep (seconds)
% p        : struct containing duty limits (p.d_min, p.d_max) and any other globals

int_err_new = int_err + err * dt;               % Update integral term
% Derivative term – guard against NaN on first iteration
if isnan(prev_err)
    deriv = 0;
else
    deriv = (err - prev_err) / dt;
end

d_unsat = Kp * err + Ki * int_err_new + Kd * deriv;   % Unclamped duty

% Anti-windup: clamp duty cycle to limits and stop integrating when saturation occurs
d = min(max(d_unsat, p.d_min), p.d_max);
if d ~= d_unsat
    % Integral freeze to prevent wind‑up
    int_err_new = int_err;
end

end
