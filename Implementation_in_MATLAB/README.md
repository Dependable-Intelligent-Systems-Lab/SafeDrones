## Implementation in MATLAB

<pre> 
% Motors Status 0 for failed and 1 for operational
% For Example for Hexacopter MotorStatus can be:
% MotorStatus = [111111] means all motors are operational and MotorStatus = [011111] means motor a has failed.
[P_Fail, MTTF] = Markov_Risk_Calc(MotorStatus, 'PNPNPN', time)
</pre>
