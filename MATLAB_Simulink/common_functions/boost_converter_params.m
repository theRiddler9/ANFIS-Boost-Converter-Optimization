function p = boost_converter_params()
%BOOST_CONVERTER_PARAMS Returns the averaged-model parameters for the
%high-voltage boost converter (20V -> 350V) used throughout the project.
%
% NOTE: L, R and switching frequency were NOT explicitly given in your
% project doc (only C = 1000 uF and Vin=20V, Vout=350V were). Replace
% p.L, p.R, p.fsw below with the exact values from your base paper once
% you have them -- everything downstream (data gen, ANFIS training,
% comparative testing) reads from this single file, so you only need to
% change it here.

p.Vin_nom = 20;        % V, nominal input voltage
p.Vref    = 350;       % V, output voltage reference (target)
p.L       = 500e-6;    % H, boost inductor  <-- CONFIRM / REPLACE
p.C       = 1000e-6;   % F, output capacitor (from base paper)
p.R       = 500;       % Ohm, nominal load resistance <-- CONFIRM / REPLACE
p.fsw     = 50e3;      % Hz, switching frequency <-- CONFIRM / REPLACE
p.dt      = 1e-6;      % s, simulation/control integration step
p.Tsim    = 0.05;      % s, duration of each test run
p.d_min   = 0;         % duty cycle saturation limits
p.d_max   = 0.97;

end
