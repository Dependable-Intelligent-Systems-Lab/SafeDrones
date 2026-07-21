# MATLAB implementation

[![View SafeDrones on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://uk.mathworks.com/matlabcentral/fileexchange/94880-safe-drones)

The [`Implementation_in_MATLAB/`](https://github.com/Dependable-Intelligent-Systems-Lab/SafeDrones/tree/master/Implementation_in_MATLAB)
directory contains standalone scripts for propulsion, battery, and GPS
reliability analysis.

## Included files

| File | Purpose |
| --- | --- |
| `Motor_Failure_Risk_Calc.m` | Configuration-aware propulsion failure probability and MTTF |
| `Battery_Failure_Risk_Calc.m` | Battery reliability assessment |
| `GPS_Failure.m` / `GPS_Failure_v2.m` | GPS availability experiments |
| `Paper_Implementation.m` | Research-model implementation |
| `Sample_Results/` | Example reliability curves |

## Propulsion example

```matlab
% 0 = failed motor, 1 = operational motor
MotorStatus = [1, 1, 1, 1, 1, 1];
Motors_Configuration = 'PNPNPN';
Lambda = 0.001;
time = 10;

[P_Fail, MTTF] = Motor_Failure_Risk_Calc( ...
    MotorStatus, Motors_Configuration, Lambda, time);
```

The ordering of `MotorStatus` must match the selected physical configuration.
The failure rate and time must also use consistent units.

## Interpreting the result

- `P_Fail` is the estimated probability of propulsion-system failure at `time`.
- `MTTF` is the mean time to failure under the model assumptions.

The result can feed mission decision logic, but aircraft-specific thresholds and
assurance arguments must be established independently.
