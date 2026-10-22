function p = boost_converter_params()
%BOOST_CONVERTER_PARAMS Returns nominal parameters for the averaged
%boost converter model used across data generation and controller testing.
%
% Nominal parameters per project specification:
%   Vin_nom = 20 V
%   Vref    = 40 V
%   L       = 500 uH
%   C       = 1000 uF
%   R       = 500 Ohm
%   rL      = 0.1 Ohm (inductor parasitic resistance)
%   fsw     = 50 kHz
%   dt      = 10 us (simulation & control discrete timestep)
%   Tsim    = 0.05 s
%   d_min   = 0.05, d_max = 0.95 (safe duty cycle limits)

p.Vin_nom = 20;        % V, nominal input voltage
p.Vref    = 40;        % V, output voltage reference (target)
p.L       = 500e-6;    % H, boost inductor
p.C       = 1000e-6;   % F, output capacitor
p.R       = 500;       % Ohm, nominal load resistance
p.rL      = 0.1;       % Ohm, inductor equivalent series resistance
p.fsw     = 50e3;      % Hz, switching frequency
p.dt      = 1e-5;      % s, simulation/control integration step (10 us)
p.Tsim    = 0.05;      % s, nominal duration of each test trajectory
p.d_min   = 0.05;      % duty cycle lower saturation limit
p.d_max   = 0.95;      % duty cycle upper saturation limit

end
