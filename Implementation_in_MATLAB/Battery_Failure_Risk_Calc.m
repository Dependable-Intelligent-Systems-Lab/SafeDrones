function [P_Fail, MTTF] = Battery_Failure_Risk_Calc(BatteryStatus, Lamdba, time, alpha, beta, battery_degradation_rate)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Program Name : Markov-based Drone's Reliability and MTTF Estimator      %
    % Author       : Koorosh Aslansefat                                       %
    % Version      : 1.0.1                                                    %
    % Description  : A Markov Process-Based Approach for Reliability          %
    %                Evaluation of the Battery System                      %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% Provided Markov model is from the following paper:
    % Kabir, S., Aslansefat, K., Sorokos, I., Papadopoulos, Y., & Gheraibia, Y. (2019, October). 
    % A conceptual framework to incorporate complex basic events in HiP-HOPS. 
    % In International Symposium on Model-Based Safety and Assessment (pp. 109-124). Springer, Cham.

    %%

    % Battery Status is an integer between 0 and 3. 0 means leass than 25% charged or fully decharged and 3 means  75% up to fully charged. 
    % Lambda: Failure Rate of the Battery System including Battery, voltage regulator and voltage/current meter.
    % alpha and beta: Are charge and discharge rates.

    %%
    syms L a b d Fp
    
    a = 0.00078;
    b = 0.00082;
    d = 0.0064;
    Fp = 0.00285;
    
    if BatteryStatus == 3
        P0 = [1 0 0 0 0 0 0 0]';
    elseif BatteryStatus == 2
        P0 = [0 0 1 0 0 0 0 0]';
    elseif BatteryStatus == 1
        P0 = [0 0 0 0 1 0 0 0]';
    else
        P_Fail = 1; 
        MTTF = 0; 
    end

    M = [-Fp-a-d,  b,       0,  0,       0,  0, 0, 0 
               a, -b,       0,  0,       0,  0, 0, 0
               d,  0, -Fp-a-d,  b,       0,  0, 0, 0
               0,  0,       a, -b,       0,  0, 0, 0 
               0,  0,       d,  0, -Fp-a-d,  b, 0, 0
               0,  0,       0,  0,       a, -b, 0, 0
               0,  0,       0,  0,       d,  0, 0, 0
              Fp,  0,      Fp,  0,      Fp,  0, 0, 0];
      
    P = expm(M*t)*P0;

    P_Fail_Symbolic  = P(end) + P(end-1);
    
    N = [-Fp-a-d,  b,       0,  0,       0,  0 
               a, -b,       0,  0,       0,  0
               d,  0, -Fp-a-d,  b,       0,  0
               0,  0,       a, -b,       0,  0 
               0,  0,       d,  0, -Fp-a-d,  b
               0,  0,       0,  0,       a, -b];
    
    Time_Symbolic = sum(sum(-1.*inv(N),2).*(P0(1:end-1)));
    
    Fp = Lamdba;
    t = time;
    d = battery_degradation_rate;
    a = alpha;
    b = beta;
    P_Fail = eval(P_Fail_Symbolic);
    MTTF = eval(Time3_Symbolic);

end
