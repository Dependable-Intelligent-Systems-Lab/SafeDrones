[![View Safe Drones on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://uk.mathworks.com/matlabcentral/fileexchange/94880-safe-drones)

## Implementation in MATLAB
<p align = 'justify'>The Markov_Risk_Calc receives the MotorStatus from a fault detection and diagnosis unit and based on the drone's configuration and its current flight time, calculates the P_fail (probability of drone failure over time) and MTTF (Mean Time To Failure). Based on these two parameters, the autonomous system can decide to a) continue the mission, b) avoid the mission and come back to station, or c) perform an emergency safe landing.</p>
<pre> 
% Motors Status: 0 for failed and 1 for operational
% For example, for a hexacopter the MotorStatus can be:
% MotorStatus = [1,1,1,1,1,1] which means all motors are operational and MotorStatus = [0,1,1,1,1,1] means motor A has failed.
% Lambda = Failure Rate of Propulsion System including rotors, motors' drivers and propellers.
% Motors_Configuration: Can be 'PNPN' for quadcopters, 'PNPNPN' and 'PPNNPN' ...
% for hexacopters and 'PPNNPPNN' for octacopter.
<a></a>
[P_Fail, MTTF] = Markov_Risk_Calc(MotorStatus, Motors_Configuration, Lambda, time)
</pre>
