%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Program Name : Markov Solver                                            %
% Author       : Koorosh Aslansefat                                       %
% Version      : 1                                                        %
% Description  : Solve Markov model of Multicopters                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Cite As: Aslansefat, Koorosh, Marques, F., Mendonça, R., & Barata, J. 
% (2019, May). A Markov Process-Based Approach for Reliability Evaluation 
%  of the Propulsion System in Multi-rotor Drones. In Doctoral Conference
%  on Computing, Electrical and Industrial Systems (pp. 91-98). Springer
%% Clear Section 
% clear
clearvars
close all
clc

%% Initialization Section
format long

Lambda_Motor = 0.01;
Lambda_Propeller = 0.008;
Lambda_Other = 0.002;
Lambda_ESC = 0.02;

Lamdba = Lambda_Motor + Lambda_Propeller + Lambda_Other + Lambda_ESC

syms t

%% Calculation Section
% M = 4
syms L t c positive 

P0=[1 0]';
M = [-4*L,  0
      4*L,  0];
P = expm(M*t)*P0;
R_Quadcopter = 1-P(end)

% M = 6
P0=[1 0 0 0 0 0]';
M = [-6*L,     0,    0,    0,    0,    0
      6*L,  -5*L,    0,    0,    0,    0
        0,     L, -4*L,    0,    0,    0
        0,   2*L,    0, -4*L,    0,    0
        0,     0,    0,    L, -3*L,    0
        0,   2*L,  4*L,  3*L,  3*L,    0];
P = expm(M*t)*P0;
R_Hexacopter = 1-P(end)

N = [-6*L,     0,    0,    0,    0
      6*L,  -5*L,    0,    0,    0
        0,     L, -4*L,    0,    0
        0,   2*L,    0, -4*L,    0
        0,     0,    0,    L, -3*L];
Time = -1.*inv(N)

P02=[1 0 0 0]';
M2 = [-6*L,     0,    0,    0
       4*L,  -5*L,    0,    0
         0,   2*L, -4*L,    0
       2*L,   3*L,  4*L,    0];
P2 = expm(M2*t)*P02;
R_Hexacopter_2 = 1-P2(end);

N2 = [-6*L,     0,    0
       4*L,  -5*L,    0
         0,   2*L, -4*L];
Time_2 = -1.*inv(N2)
sum(Time_2,1);
%% Plotting Section
s = figure(1);
set(s,'Color','w');

t = 0:50;
R_Quadcopter = exp(-4*Lamdba*t);
R_Quadcopter_2 = exp(-4*Lamdba*t);
R_Hexacopter = 2*exp(-3*Lamdba*t) + 3*exp(-4*Lamdba*t) - 6*exp(-5*Lamdba*t) + 2*exp(-6*Lamdba*t);
R_Hexacopter_P = 2*exp(-6*Lamdba*t) - 6*exp(-5*Lamdba*t) + 3*exp(-4*Lamdba*t) + 2*exp(-3*Lamdba*t);

R_Hexacopter_2 = 4*exp(-4*Lamdba*t) - 4*exp(-5*Lamdba*t) + exp(-6*Lamdba*t);
R_Hexacopter_P2 = -2*exp(-5*Lamdba*t) + 3*exp(-4*Lamdba*t);

plot(t,R_Quadcopter,'--r','LineWidth',2)

ent = sprintf('\n');
title(['\fontsize{13}Reliability of Multicopter' ent '\fontsize{13} Exponential Failure Distribution with \lambda = 0.04'])
xlabel('\fontsize{11}Time (hours)')
ylabel('\fontsize{11}Reliability')
hold on
plot(t,R_Hexacopter,'-.b','LineWidth',2)
% plot(N,P3.*N,':m','LineWidth',2)
% plot(N,P4.*N,'--k','LineWidth',2)
plot(t,R_Hexacopter_2,':m','LineWidth',2)
plot(t,R_Quadcopter,'sk','LineWidth',2)
plot(t,R_Hexacopter_P,'oy','LineWidth',2)
plot(t,R_Hexacopter_P2,'^g','LineWidth',2)

hleg = legend('\fontsize{11}Quad-copter','\fontsize{11}Hexa-copter | PNPNPN'...
              ,'\fontsize{11}Hexa-copter | PPNNPN','\fontsize{11}Quad-copter (D. Shi et al. 2016)','\fontsize{11}Hexa-copter | PNPNPN (D. Shi et al. 2016)','\fontsize{11}Hexa-copter | PPNNPN (D. Shi et al. 2016)','Location','Northeast');
       set(hleg,'FontAngle','italic','TextColor',[.3,.2,.1])
hold off

t = [5, 10, 15, 20, 25, 30, 35];
R_Hexacopter = 2*exp(-3*Lamdba*t) + 3*exp(-4*Lamdba*t) - 6*exp(-5*Lamdba*t) + 2*exp(-6*Lamdba*t);
%%
[ 11/(30*Lamdba), 3/(10*Lamdba), 1/(4*Lamdba)];
%% Plotting Section #2
s = figure(2);
set(s,'Color','w');

t = 2;
Lamdba = 0.01:0.01:0.5;
R_Quadcopter = exp(-4*Lamdba*t);
% R_Quadcopter_2 = exp(-4*Lamdba*t);
R_Hexacopter = 2*exp(-3*Lamdba*t) + 3*exp(-4*Lamdba*t) - 6*exp(-5*Lamdba*t) + 2*exp(-6*Lamdba*t);
% R_Hexacopter_P = 2*exp(-6*Lamdba*t) - 6*exp(-5*Lamdba*t) + 3*exp(-4*Lamdba*t) + 2*exp(-3*Lamdba*t);

R_Hexacopter_2 = 4*exp(-4*Lamdba*t) - 4*exp(-5*Lamdba*t) + exp(-6*Lamdba*t);
% R_Hexacopter_P2 = -2*exp(-5*Lamdba*t) + 3*exp(-4*Lamdba*t);

plot(Lamdba,R_Quadcopter,'--r','LineWidth',2)

ent = sprintf('\n');
title(['\fontsize{13}Reliability of Multicopter vs. Failure rate of each rotor'])
xlabel('\fontsize{11}Failure rate of each rotor (\lambda)')
ylabel('\fontsize{11}Reliability of Multicopter')
hold on
plot(Lamdba,R_Hexacopter,'-.b','LineWidth',2)
% plot(N,P3.*N,':m','LineWidth',2)
% plot(N,P4.*N,'--k','LineWidth',2)
plot(Lamdba,R_Hexacopter_2,':m','LineWidth',2)
% plot(t,R_Quadcopter,'sk','LineWidth',2)
% plot(t,R_Hexacopter_P,'oy','LineWidth',2)
% plot(t,R_Hexacopter_P2,'^g','LineWidth',2)

hleg = legend('\fontsize{11}Quad-copter','\fontsize{11}Hexa-copter | PNPNPN'...
              ,'\fontsize{11}Hexa-copter | PPNNPN','\fontsize{11}Quad-copter (D. Shi et al. 2016)','\fontsize{11}Hexa-copter | PNPNPN (D. Shi et al. 2016)','\fontsize{11}Hexa-copter | PPNNPN (D. Shi et al. 2016)','Location','Northeast');
       set(hleg,'FontAngle','italic','TextColor',[.3,.2,.1])
hold off
