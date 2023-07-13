close all
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

% 13 sats out of 14 sats are available
P0 = [0, 1, 0, 0, 0, 0, 0, 0, 0]';
[P_Fail13, MTTF13] = GPS_Failure_2(P0, L, t);
plot(t, P_Fail13, 'LineWidth',2)

% 12 sats out of 14 sats are available
P0 = [0, 0, 1, 0, 0, 0, 0, 0, 0]';
[P_Fail12, MTTF12] = GPS_Failure_2(P0, L, t);
plot(t, P_Fail12, 'LineWidth',2)

% 11 sats out of 14 sats are available
P0 = [0, 0, 0, 1, 0, 0, 0, 0, 0]';
[P_Fail11, MTTF11] = GPS_Failure_2(P0, L, t);
plot(t, P_Fail11, 'LineWidth',2)

% 10 sats out of 14 sats are available
P0 = [0, 0, 0, 0, 1, 0, 0, 0, 0]';
[P_Fail10, MTTF10] = GPS_Failure_2(P0, L, t);
plot(t, P_Fail10, 'LineWidth',2)

% 9 sats out of 14 sats are available
P0 = [0, 0, 0, 0, 0, 1, 0, 0, 0]';
[P_Fail9, MTTF9] = GPS_Failure_2(P0, L, t);
plot(t, P_Fail9, 'LineWidth',2)

% 8 sats out of 14 sats are available
P0 = [0, 0, 0, 0, 0, 0, 1, 0, 0]';
[P_Fail8, MTTF8] = GPS_Failure_2(P0, L, t);
plot(t, P_Fail8, 'LineWidth',2)

% 7 sats out of 14 sats are available
P0 = [0, 0, 0, 0, 0, 0, 0, 1, 0]';
[P_Fail7, MTTF7] = GPS_Failure_2(P0, L, t);
plot(t, P_Fail7, 'LineWidth',2)

% F sats out of 14 sats are available
P0 = [0, 0, 0, 0, 0, 0, 0, 0, 1]';
[P_Fail, MTTF] = GPS_Failure_2(P0, L, t);

legend('14 sats out of 14 sats are available', '13 sats out of 14 sats are available', ...
       '12 sats out of 14 sats are available', '11 sats out of 14 sats are available', ...
       '10 sats out of 14 sats are available', '09 sats out of 14 sats are available', ...
       '08 sats out of 14 sats are available', '07 sats out of 14 sats are available', ...
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