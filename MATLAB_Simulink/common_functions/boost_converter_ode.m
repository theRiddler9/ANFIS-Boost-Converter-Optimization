function dx = boost_converter_ode(x, d, Vin, p)
%BOOST_CONVERTER_ODE Averaged (continuous-conduction-mode) model of a
%boost converter.
%
% States:   x = [iL; vC]   (inductor current, output/capacitor voltage)
% Inputs:   d   = duty cycle (0-1)
%           Vin = instantaneous input voltage (V)
%           p   = parameter struct from boost_converter_params()
%
% Equations:
%   L * diL/dt = Vin - (1-d) * vC
%   C * dvC/dt = (1-d) * iL - vC / R

iL = x(1);
vC = x(2);

diL = (Vin - (1 - d) * vC) / p.L;
dvC = ((1 - d) * iL - vC / p.R) / p.C;

dx = [diL; dvC];

end
