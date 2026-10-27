function log = simulate_closed_loop(controller_type, controller_data, Vin_profile, p, R_profile, Vref_profile)
%SIMULATE_CLOSED_LOOP Runs a unified simulation of the CCM averaged boost
%converter under closed-loop control with IDENTICAL duty limits and input filtering.
%
%   log = simulate_closed_loop(controller_type, controller_data, Vin_profile, p, R_profile, Vref_profile)
%
% Inputs:
%   controller_type : 'PID' | 'FUZZY' | 'ANFIS'
%   controller_data : for PID -> struct with fields Kp, Ki, Kd (and optional base_duty)
%                     for FUZZY -> empty or struct/FIS (calls fuzzy_controller)
%                     for ANFIS -> struct with params1, params2, consequents OR path to .fis
%   Vin_profile     : function handle @(t) -> instantaneous Vin in Volts
%   p               : parameter struct from boost_converter_params()
%   R_profile       : (optional) function handle @(t) -> instantaneous R in Ohms (default: @(t) p.R)
%   Vref_profile    : (optional) function handle @(t) -> instantaneous Vref in Volts (default: @(t) p.Vref)
%
% Output:
%   log struct with fields:
%     t, vC, iL, d, err, der, Vin, R_load, Vref

if nargin < 5 || isempty(R_profile)
    R_profile = @(t) p.R;
end
if nargin < 6 || isempty(Vref_profile)
    Vref_profile = @(t) p.Vref;
end

N = round(p.Tsim / p.dt);
t    = (0:N-1) * p.dt;
iL   = zeros(1, N);
vC   = zeros(1, N);
d    = zeros(1, N);
err  = zeros(1, N);
der  = zeros(1, N);
Vin_arr  = zeros(1, N);
R_arr    = zeros(1, N);
Vref_arr = zeros(1, N);

% Initial states: x = [iL; vC] unenergized
x = [0; 0];
int_err  = 0;
prev_err = 0;
prev_der = 0;

% Derivative filter parameters: s / (tau_f*s + 1)
tau_f = 0.002;
if isfield(p, 'tau_f')
    tau_f = p.tau_f;
end
alpha_f = tau_f / (tau_f + p.dt);
beta_f  = 1.0 / (tau_f + p.dt);

for k = 1:N
    tk     = t(k);
    Vin_k  = Vin_profile(tk);
    Vref_k = Vref_profile(tk);
    R_k    = R_profile(tk);
    
    e = Vref_k - x(2);
    
    % Unified filtered derivative calculation
    if k == 1
        de_dt = 0;
    else
        de_raw = e - prev_err;
        de_dt  = alpha_f * prev_der + beta_f * de_raw;
    end

    switch upper(controller_type)
        case 'PID'
            base_duty = 0.5;
            if isfield(controller_data, 'base_duty')
                base_duty = controller_data.base_duty;
            end
            [dk, int_err] = pid_controller(e, int_err, de_dt, ...
                controller_data.Kp, controller_data.Ki, ...
                controller_data.Kd, p.dt, p, base_duty);

        case 'FUZZY'
            dk = fuzzy_controller(e, de_dt, p);

        case 'ANFIS'
            dk = anfis_eval(e, de_dt, controller_data, p);

        otherwise
            error('Unknown controller_type: %s', controller_type);
    end

    % Integrate forward one step via averaged CCM model
    dx = boost_converter_ode(x, dk, Vin_k, p, R_k);
    x  = x + p.dt * dx;

    % Physical boundary protection (no negative capacitor voltage or inductor current)
    x(1) = max(0, x(1));
    x(2) = max(0, x(2));

    iL(k)   = x(1);
    vC(k)   = x(2);
    d(k)    = dk;
    err(k)  = e;
    der(k)  = de_dt;
    Vin_arr(k)  = Vin_k;
    R_arr(k)    = R_k;
    Vref_arr(k) = Vref_k;

    prev_err = e;
    prev_der = de_dt;
end

log.t        = t;
log.iL       = iL;
log.vC       = vC;
log.d        = d;
log.err      = err;
log.der      = der;
log.Vin      = Vin_arr;
log.R_load   = R_arr;
log.Vref     = Vref_arr;

end
