# ANFIS_Boost_Converter

Adaptive Neuro-Fuzzy Controller for a High-Voltage DC-DC Boost Converter (20V -> 350V).

## What's already coded (no Simulink needed to run these)

All of the following use a pure-MATLAB averaged state-space model of the
boost converter (`common_functions/boost_converter_ode.m`), so you can run
the entire pipeline **today**, before your `.slx` models exist:

1. `MATLAB_Simulink/03_Manual_Fuzzy_Controller/build_fuzzy_fis.m`
   Builds the 49-rule hand-designed fuzzy controller, saves `fuzzy_49rules.fis`.

2. `MATLAB_Simulink/04_Ideal_Controller_DataGen/generate_training_data.m`
   Runs an aggressive "ideal teacher" PID through disturbance scenarios,
   logs [error, d_error, duty] -> saves `Data/Training_Datasets/anfis_training_data.mat`.

3. `MATLAB_Simulink/05_ANFIS_Training/train_anfis.m`
   Trains the ANFIS model on that dataset -> saves `anfis_model.fis`
   and a training-convergence plot.

4. `MATLAB_Simulink/07_Comparative_Testing/run_all_controllers.m`
   and `disturbance_test_script.m`
   Runs PID vs Fuzzy vs ANFIS on identical disturbance tests, computes
   response time / overshoot / settling time / steady-state error, saves
   plots + CSV tables to `Results_Plots/` and `Data/Results/`.

### Run order
```
build_fuzzy_fis.m
generate_training_data.m
train_anfis.m
run_all_controllers.m
disturbance_test_script.m
```

## What still needs building

`01_Boost_Converter_Model`, `02_PID_Controller`, and `6_ANFIS_Deployment`
are placeholders for your actual **Simulink circuit models** (switch,
diode, inductor, capacitor via Simscape Electrical). Once those `.slx`
files exist, you can swap the pure-MATLAB `simulate_closed_loop()` calls
for `sim('your_model.slx')` calls with minimal changes to the rest of the
pipeline — the controllers, training data format, and metrics code all
stay the same either way.

## Before trusting the numbers

Check `common_functions/boost_converter_params.m` — `L`, `R`, and `fsw`
are placeholders since your project doc only specified `C = 1000 µF`,
`Vin = 20V`, `Vout = 350V`. Replace with your base paper's actual values.
