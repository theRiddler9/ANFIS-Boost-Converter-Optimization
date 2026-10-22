function dx = boost_converter_ode(x, d, Vin, p, R_load)
%BOOST_CONVERTER_ODE Averaged continuous-conduction-mode (CCM) model of a
%DC-DC boost converter including inductor parasitic resistance.
%
% States:   x = [iL; vC]   (inductor current in A, capacitor/output voltage in V)
% Inputs:   d      = duty cycle command (0-1)
%           Vin    = instantaneous input voltage (V)
%           p      = parameter struct from boost_converter_params()
%           R_load = (optional) instantaneous load resistance (Ohm). Defaults to p.R.
%
% Governing Averaged Equations:
%   L * diL/dt = Vin - (1-d) * vC - rL * iL
%   C * dvC/dt = (1-d) * iL - vC / R

if nargin < 5 || isempty(R_load)
    R_load = p.R;
end

rL = 0;
if isfield(p, 'rL')
    rL = p.rL;
end

iL = x(1);
vC = x(2);

diL = (Vin - (1 - d) * vC - rL * iL) / p.L;
dvC = ((1 - d) * iL - vC / R_load) / p.C;

dx = [diL; dvC];

end
