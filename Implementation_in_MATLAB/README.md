## Implementation in MATLAB
<p align = 'justify'>The Markov_Risk_Calc receive the Motorstatus from fault detection and diagnosis unit and based on the drones configuration and its current flight time, calculate the P_fail and MTTF. Where P_Fail is the probability of drone being failed vs time and MTTF also stands for Mean Time To Failure. Based on these two parameters the autonomous system can decide to I) continue the mission, II) avoid the mission and come back to station or III) do emergency safe landing.</p>
<pre> 
% Motors Status 0 for failed and 1 for operational
% For Example for Hexacopter MotorStatus can be:
% MotorStatus = [111111] means all motors are operational and MotorStatus = [011111] means motor a has failed.
% Failure Rate of Propulsion System including rotors, motors' drivers and propellers.

[P_Fail, MTTF] = Markov_Risk_Calc(MotorStatus, 'PNPNPN', time)
</pre>
