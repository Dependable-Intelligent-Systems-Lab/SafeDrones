Sample Code:

```
# Motors Status 0 for failed and 1 for operational
# For Example for Hexacopter MotorStatus can be:
# MotorStatus = [1,1,1,1,1,1] means all motors are operational and MotorStatus = [0,1,1,1,1,1] means motor a has failed.
# Lambda = Failure Rate of Propulsion System including rotors, motors' drivers and propellers.
# Motors_Configuration: It can be 'PNPN' for quadcopters, 'PNPNPN' and 'PPNNPN' for hexacopters and 'PPNNPPNN' for octacopter.

P_Fail, MTTF = Motor_Failure_Risk_Calc([1,1,1,1,1,1], 'PPNNPN', 0.01, 100)
```
