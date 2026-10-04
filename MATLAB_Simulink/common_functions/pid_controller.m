function [d, int_err_new] = pid_controller(err, int_err, prev_err, Kp, Ki, Kd, dt, p)
%PID_CONTROLLER Discrete PID duty-cycle controller.
%
%   [d, int_err_new] = pid_controller(err, int_err, prev_err, Kp, Ki, Kd, dt, p)
%
% err      : current error (Vref - Vout)
% int_err  : accumulated integral of error so far
% prev_err : error at the previous timestep (for derivative term)
% Kp,Ki,Kd : PID gains
% dt       : timestep (s)
% p        : parameter struct (for duty cycle saturation limits)

int_err_new = int_err + err * dt;
deriv       = (err - prev_err) / dt;

d_unsat = Kp * err + Ki * int_err_new + Kd * deriv;

% Anti-windup: clamp duty cycle, and freeze integral if saturated
d = min(max(d_unsat, p.d_min), p.d_max);
if d ~= d_unsat
    int_err_new = int_err; % stop integrating further while saturated
end

end
