function [d, int_err_new] = pid_controller(err, int_err, deriv, Kp, Ki, Kd, dt, p, base_duty)
%PID_CONTROLLER Discrete anti-windup PID duty-cycle controller.
%
%   [d, int_err_new] = pid_controller(err, int_err, deriv, Kp, Ki, Kd, dt, p, base_duty)
%
% err       : current error (Vref - Vout) in Volts
% int_err   : accumulated integral of error so far (V*s)
% deriv     : error derivative (de/dt) in V/s
% Kp,Ki,Kd  : PID gains
% dt        : integration timestep (s)
% p         : parameter struct (d_min, d_max)
% base_duty : (optional) feedforward nominal duty cycle (default: 0.5)

if nargin < 9 || isempty(base_duty)
    base_duty = 0.5;
end

int_err_cand = int_err + err * dt;
d_unsat = base_duty + Kp * err + Ki * int_err_cand + Kd * deriv;

% Anti-windup: clamp duty cycle to identical limits [d_min, d_max]
d = min(max(d_unsat, p.d_min), p.d_max);

% Freeze integral if saturated in direction that worsens saturation
if (d_unsat >= p.d_max && err > 0) || (d_unsat <= p.d_min && err < 0)
    int_err_new = int_err;
else
    int_err_new = int_err_cand;
end

end
