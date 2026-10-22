function log = simulate_closed_loop(controller_type, controller_data, Vin_profile, p)
%SIMULATE_CLOSED_LOOP Runs a fixed-step Euler simulation of the boost
%converter under closed-loop control, for ANY controller type.
%
%   log = simulate_closed_loop(controller_type, controller_data, Vin_profile, p)
%
% controller_type : 'PID' | 'FUZZY' | 'ANFIS'
% controller_data : struct for PID -> fields Kp,Ki,Kd
%                   for FUZZY/ANFIS -> the loaded FIS object (readfis output)
% Vin_profile      : function handle @(t) -> instantaneous Vin (V), lets
%                    you simulate step disturbances e.g. 20V -> 25V at t=0.02s
% p                : parameter struct from boost_converter_params()
%
% Returns log struct with fields: t, vC, iL, d, err

N = round(p.Tsim / p.dt);
t   = (0:N-1) * p.dt;
iL  = zeros(1, N);
vC  = zeros(1, N);
d   = zeros(1, N);
err = zeros(1, N);

x = [0; 0];           % start from zero initial conditions (converter off)
int_err  = 0;
prev_err = 0;

for k = 1:N
    Vin = Vin_profile(t(k));
    e   = p.Vref - x(2);
    de = e - prev_err;

switch upper(controller_type)
    case 'PID'
        [dk, int_err] = pid_controller(e, int_err, prev_err, ...
            controller_data.Kp, controller_data.Ki, ...
            controller_data.Kd, p.dt, p);

    case 'FUZZY'
        fuzzyError = min(max(e, -50), 50);
        fuzzyDError = min(max(de, -500), 500);

        dk = evalfis(controller_data, ...
            [fuzzyError, fuzzyDError]);
        dk = min(max(dk, p.d_min), p.d_max);

    case 'ANFIS'
        % Use the same feature definition as the training dataset.
        anfisError = min(max(e, -12.595), 26.03);
        anfisDError = min(max(de, -4.9991), 5.0009);

        dk = evalfis(controller_data, ...
            [anfisError, anfisDError]);
        dk = min(max(dk, p.d_min), p.d_max);

    otherwise
        error('Unknown controller_type: %s', controller_type);
end

    % Integrate one step (simple fixed-step Euler is fine at dt=1us)
    dx = boost_converter_ode(x, dk, Vin, p);
    x  = x + p.dt * dx;

    iL(k)  = x(1);
    vC(k)  = x(2);
    d(k)   = dk;
    err(k) = e;

    prev_err = e;
end

log.t   = t;
log.iL  = iL;
log.vC  = vC;
log.d   = d;
log.err = err;

end
