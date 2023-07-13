close all
clearvars
clc

L = 0.004;
t = 1:500;

% 14 sats out of 14 sats are available
P0 = [1, 0, 0, 0, 0, 0, 0, 0, 0]';
[P_Fail14, MTTF14] = GPS_Failure_2(P0, L, t);

plot(t, P_Fail14, 'LineStyle','--', 'LineWidth',2)
hold on
xlabel('Time')
ylabel('Probability of Failure')

t1 = 1:100;

% 10 sats out of 14 sats are available
P0 = [0, 0, 0, 0, 1, 0, 0, 0, 0]';
[P_Fail10, MTTF10] = GPS_Failure_2(P0, L, t1);
plot([t1], [P_Fail10], 'LineWidth',2)

t2 = 100:200;

% 9 sats out of 14 sats are available
P0 = [0, 0, 0, 0, 0, 1, 0, 0, 0]';
[P_Fail9, MTTF9] = GPS_Failure_2(P0, L, t2);
plot([t1(end), t2], [P_Fail10(end), P_Fail9], 'LineWidth',2)

t3 = 200:300;
% 13 sats out of 14 sats are available
P0 = [0, 1, 0, 0, 0, 0, 0, 0, 0]';
[P_Fail13, MTTF13] = GPS_Failure_2(P0, L, t3);
plot([t2(end), t3], [P_Fail9(end), P_Fail13], 'LineWidth',2)

t4 = 300:400;
% 7 sats out of 14 sats are available
P0 = [0, 0, 0, 0, 0, 0, 0, 1, 0]';
[P_Fail7, MTTF7] = GPS_Failure_2(P0, L, t4);
plot([t3(end), t4], [P_Fail13(end), P_Fail7], 'LineWidth',2)

t5 = 400:500;
P0 = [1, 0, 0, 0, 0, 0, 0, 0, 0]';
[P_Fail142, MTTF142] = GPS_Failure_2(P0, L, t5);
plot([t4(end), t5], [P_Fail7(end), P_Fail142], 'LineWidth',2)



legend('14 sats out of 14 sats are available', '10 sats out of 14 sats are available', ...
       '09 sats out of 14 sats are available', '13 sats out of 14 sats are available', ...
       '07 sats out of 14 sats are available', '14 sats out of 14 sats are available', ...
       'Location','best')

function [P_Fail, MTTF] = GPS_Failure_2(P0, Lambda, time)
    syms L t

    M = [-14*L,     0,     0,     0,     0,     0,     0,     0,    0
          14*L, -13*L,     0,     0,     0,     0,     0,     0,    0
            0,   13*L, -12*L,     0,     0,     0,     0,     0,    0
            0,      0,  12*L, -11*L,     0,     0,     0,     0,    0
            0,      0,     0,  11*L, -10*L,     0,     0,     0,    0
            0,      0,     0,     0,  10*L,  -9*L,     0,     0,    0
            0,      0,     0,     0,     0,   9*L,  -8*L,     0,    0
            0,      0,     0,     0,     0,     0,   8*L,  -7*L,    0
            0,      0,     0,     0,     0,     0,     0,   7*L,    0];
    
    P = expm(M*t)*P0;
    P_Fail_Symbolic = P(end);
    
    N = [-14*L,     0,     0,     0,     0,     0,     0,     0
          14*L, -13*L,     0,     0,     0,     0,     0,     0
            0,   13*L, -12*L,     0,     0,     0,     0,     0
            0,      0,  12*L, -11*L,     0,     0,     0,     0
            0,      0,     0,  11*L, -10*L,     0,     0,     0
            0,      0,     0,     0,  10*L,  -9*L,     0,     0
            0,      0,     0,     0,     0,   9*L,  -8*L,     0
            0,      0,     0,     0,     0,     0,   8*L,  -7*L];

    Time_Symbolic = sum(sum(-1.*inv(N),2).*(P0(1:8)));
    
    L = Lambda;
    t = time;
    
    P_Fail = eval(P_Fail_Symbolic);
    MTTF = eval(Time_Symbolic);

end