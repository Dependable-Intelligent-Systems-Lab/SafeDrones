function [P_Fail, MTTF] = Motor_Failure_Risk_Calc(MotorStatus, Motors_Configuration, Lamdba, time)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Program Name : Markov-based Drone's Reliability and MTTF Estimator      %
    % Author       : Koorosh Aslansefat                                       %
    % Version      : 1.0.1                                                    %
    % Description  : A Markov Process-Based Approach for Reliability          %
    %                Evaluation of the Propulsion System                      %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% Cite As: Aslansefat, Koorosh, Marques, F., Mendonça, R., & Barata, J. 
    % (2019, May). A Markov Process-Based Approach for Reliability Evaluation 
    %  of the Propulsion System in Multi-rotor Drones. In Doctoral Conference
    %  on Computing, Electrical and Industrial Systems (pp. 91-98). Springer

    %%

    % Motors Status 0 for failed and 1 for operational
    % For Example for Hexacopter MotorStatus can be:
    % MotorStatus = [1,1,1,1,1,1] means all motors are operational and MotorStatus = [0,1,1,1,1,1] means motor a has failed.
    % Lambda = Failure Rate of Propulsion System including rotors, motors' drivers and propellers.
    % Motors_Configuration: It can be 'PNPN' for quadcopters, 'PNPNPN' and 'PPNNPN' ...
    % for hexacopters and 'PPNNPPNN' for octacopter.

    %%
    syms L t
    
    SysState = sum(MotorStatus);

    switch Motors_Configuration

        case 'PNPN'
            if numel(MotorStatus) > 4
               disp('The MotorStatus vector should match Motors_Configuration vector') 
            else
                if SysState < 4
                   P_Fail = 1; 
                   MTTF = 0;
                else
                   P_Fail = 1 - exp(-4*Lamdba*time); 
                   MTTF = 1/(4*Lamdba);
                end
            end

        case 'PNPNPN'
            if SysState == 6
                P0=[1 0 0 0 0 0]';
            elseif SysState == 5
                P0=[0 1 0 0 0 0]';
            elseif SysState == 4
                if (MotorStatus(1)+MotorStatus(4)) == 0 || (MotorStatus(2)+MotorStatus(5)) == 0 || (MotorStatus(3)+MotorStatus(6)) == 0
                   P0=[0 0 1 0 0 0]'; 
                else
                   P0=[0 0 0 1 0 0]';  
                end
            elseif SysState == 3
                if (MotorStatus(1)+MotorStatus(4)) == 0 || (MotorStatus(2)+MotorStatus(5)) == 0 || (MotorStatus(3)+MotorStatus(6)) == 0
                   P_Fail = 1; 
                   MTTF = 0; 
                else
                   P0=[0 0 0 0 1 0]'; 
                end
            else
                P_Fail = 1; 
                MTTF = 0; 
            end 

            M = [-6*L,     0,    0,    0,    0,    0
                  6*L,  -5*L,    0,    0,    0,    0
                    0,     L, -4*L,    0,    0,    0
                    0,   2*L,    0, -4*L,    0,    0
                    0,     0,    0,    L, -3*L,    0
                    0,   2*L,  4*L,  3*L,  3*L,    0];

            P = expm(M*t)*P0;
            P_Fail_Symbolic = P(end);

            N = [-6*L,     0,    0,    0,    0
                  6*L,  -5*L,    0,    0,    0
                    0,     L, -4*L,    0,    0
                    0,   2*L,    0, -4*L,    0
                    0,     0,    0,    L, -3*L];
            Time_Symbolic = sum(sum(-1.*inv(N),2).*(P0(1:5)));

            L = Lamdba;
            t = time;
            P_Fail = eval(P_Fail_Symbolic);
            MTTF = eval(Time_Symbolic);


        case 'PPNNPN'
            if SysState == 6
               P02=[1 0 0 0]';
            elseif SysState == 5
               P02=[0 1 0 0]'; 
            elseif SysState == 4
               P02=[0 0 1 0]'; 
            else
                P_Fail = 1; 
                MTTF = 0;
            end

            syms L t

            M2 = [-6*L,     0,    0,    0
                   4*L,  -5*L,    0,    0
                     0,   2*L, -4*L,    0
                   2*L,   3*L,  4*L,    0];
            P2 = expm(M2*t)*P02;
            P_Fail2_Symbolic = P2(end);

            N2 = [-6*L,     0,    0
                   4*L,  -5*L,    0
                    0,   2*L, -4*L];
            Time2_Symbolic = sum(sum(-1.*inv(N2),2).*(P02(1:3)));

            L = Lamdba;
            t = time;
            P_Fail = eval(P_Fail2_Symbolic);
            MTTF = eval(Time2_Symbolic);
            
        case 'PPNNPPNN'
            if SysState == 8
                P03=[1 0 0 0 0 0]';
            elseif SysState == 7
                P03=[0 1 0 0 0 0]';
            elseif SysState == 6
                P03=[0 0 1 0 0 0]';
            elseif SysState == 5
                P03=[0 0 0 1 0 0]';
            elseif SysState == 4  
                P03=[0 0 0 0 1 0]';
            else
                P_Fail = 1; 
                MTTF = 0; 
            end

            M3 = [-8*L,     0,    0,    0,    0,    0
                   8*L,  -7*L,    0,    0,    0,    0
                     0,   6*L, -6*L,    0,    0,    0
                     0,     0,  4*L, -5*L,    0,    0
                     0,     0,    0,  2*L, -4*L,    0
                     0,   1*L,  2*L,  3*L,  4*L,    0];

            P3 = expm(M3*t)*P03;
            P_Fail3_Symbolic = P3(end);

            N3 = [-8*L,     0,    0,    0,    0
                   8*L,  -7*L,    0,    0,    0
                     0,   6*L, -6*L,    0,    0
                     0,     0,  4*L, -5*L,    0
                     0,     0,    0,  2*L, -4*L];
            Time3_Symbolic = sum(sum(-1.*inv(N3),2).*(P03(1:5)));

            L = Lamdba;
            t = time;
            P_Fail = eval(P_Fail3_Symbolic);
            MTTF = eval(Time3_Symbolic);

        otherwise
            disp('The current Motors_Configuration is not defined in this version, please check the updated versions')
    end

end
