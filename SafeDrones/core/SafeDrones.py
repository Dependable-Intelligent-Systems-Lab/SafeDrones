import numpy as np
import pandas as pd # data processing, CSV file I/O (e.g. pd.read_csv)
import sympy as sym

class SafeDrones:
    
    def __init__(self):
        
        # Observasions or Symptoms from Diagnosis System
        self.MotorStatus = None
        self.CommStatus = None
        self.BattStatus = None
        
        # Drone Settings
        self.Motors_Configuration = None
        
        # Defining Failure Rates
        self.Motors_Lambda = None
        self.Comm_Lambda = None
        self.Batt_Lambda = None
        
        # Mission Time
        self.time = None
        

    def Motor_Failure_Risk_Calc(self, MotorStatus, Motors_Configuration, Motors_Lambda, time):

        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # Program Name : Markov-based Drone's Reliability and MTTF Estimator      %
        # Author       : Koorosh Aslansefat                                       %
        # Version      : 1.0.0                                                    %
        # Description  : A Markov Process-Based Approach for Reliability          %
        #                Evaluation of the Propulsion System for Drones           %
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        '''
        Cite As: Aslansefat, Koorosh, Marques, F., Mendonça, R., & Barata, J. 
         (2019, May). A Markov Process-Based Approach for Reliability Evaluation 
          of the Propulsion System in Multi-rotor Drones. In Doctoral Conference
          on Computing, Electrical and Industrial Systems (pp. 91-98). Springer

        Motors Status 0 for failed and 1 for operational
        For Example for Hexacopter MotorsStatus can be:
        MotorsStatus = [1,1,1,1,1,1] means all motors are operational and MotorsStatus = [0,1,1,1,1,1] means motor a has failed.
        Lambda = Failure Rate of Propulsion System including rotors, motors' drivers and propellers.
        Motors_Configuration: It can be 'PNPN' for quadcopters, 'PNPNPN' and 'PPNNPN' for hexacopters and 'PPNNPPNN' for octacopter.
        '''

        ##
        import numpy as np # linear algebra
        import pandas as pd # data processing, CSV file I/O (e.g. pd.read_csv)
        import sympy as sym

        L = sym.Symbol('L')
        t = sym.Symbol('t')
        
        SysState = sum(MotorStatus)

        if  Motors_Configuration == 'PNPN':
            if len(MotorStatus) > 4:
                   disp('The MotorStatus vector should match Motors_Configuration vector') 
            else:
                if SysState < 4:
                    P_Fail = 1
                    MTTF = 0
                else:
                    P_Fail = 1 - sym.exp(-4*L*t)
                    MTTF = 1/(4*L)  
                    
        elif Motors_Configuration == 'PNPNPN':
            
            if SysState == 6:
                P0 = sym.Matrix([[1],[0],[0],[0],[0],[0]])
                Sflag = 4
            elif SysState == 5:
                P0 = sym.Matrix([[0],[1],[0],[0],[0],[0]])
                Sflag = 3
            elif SysState == 4:
                if (MotorStatus[0]+MotorStatus[3]) == 0 or (MotorStatus[1]+MotorStatus[4]) == 0 or (MotorStatus[2]+MotorStatus[5]) == 0:
                    P0 = sym.Matrix([[0],[0],[1],[0],[0],[0]])
                    Sflag = 2
                else:
                    P0 = sym.Matrix([[0],[0],[0],[1],[0],[0]])
                    Sflag = 1

            elif SysState == 3:
                if (MotorStatus[0]+MotorStatus[3]) == 0 or (MotorStatus[1]+MotorStatus[4]) == 0 or (MotorStatus[2]+MotorStatus[5]) == 0:
                    P_Fail = 1
                    MTTF = 0; 
                else:
                    P0 = sym.Matrix([[0],[0],[0],[0],[1],[0]])
                    Sflag = 0
            else:
                P_Fail = 1
                MTTF = 0

            M = sym.Matrix([[-6*L,     0,    0,    0,    0,    0],
                                [ 6*L,  -5*L,    0,    0,    0,    0],
                                [   0,     L, -4*L,    0,    0,    0],
                                [   0,   2*L,    0, -4*L,    0,    0],
                                [   0,     0,    0,    L, -3*L,    0],
                                [   0,   2*L,  4*L,  3*L,  3*L,    0]])


            P = sym.exp(M*t)*P0
            P_Fail = P[-1]
            
            N = sym.Matrix([[-6*L,     0,    0,    0,    0],
                                [ 6*L,  -5*L,    0,    0,    0],
                                [   0,     L, -4*L,    0,    0],
                                [   0,   2*L,    0, -4*L,    0],
                                [   0,     0,    0,    L, -3*L]])
            tt = -1*N.inv()
            MTTF = sum(tt[Sflag,:])
        
        elif  Motors_Configuration == 'PPNNPN':
            
            if SysState == 6:
                P02 = sym.Matrix([[1],[0],[0],[0]])
                Sflag = 2
            elif SysState == 5:
                P02 = sym.Matrix([[0],[1],[0],[0]])
                Sflag = 1
            elif SysState == 4:
                P02 = sym.Matrix([[0],[0],[1],[0]])
                Sflag = 0
            else:
                P_Fail = 1; 
                MTTF = 0;

            M2 = sym.Matrix([[-6*L,     0,    0,    0],
                             [  4*L,  -5*L,    0,    0],
                             [    0,   2*L, -4*L,    0],
                             [  2*L,   3*L,  4*L,    0]])
            P2 = sym.exp(M2*t)*P02
            P_Fail = P2[-1]

            N2 = sym.Matrix([[-6*L,     0,    0],
                              [4*L,  -5*L,    0],
                              [  0,   2*L, -4*L]])
                
            tt = -1*N2.inv()
            MTTF = sum(tt[Sflag,:])
        elif Motors_Configuration == 'PPNNPPNN':

            if SysState == 8:
                P03 = sym.Matrix([[1],[0],[0],[0],[0],[0]])
                Sflag = 4
            elif SysState == 7:
                P03 = sym.Matrix([[0],[1],[0],[0],[0],[0]])
                Sflag = 3
            elif SysState == 6:
                P03 = sym.Matrix([[0],[0],[1],[0],[0],[0]])
                Sflag = 2
            elif SysState == 5:
                P03 = sym.Matrix([[0],[0],[0],[1],[0],[0]])
                Sflag = 1
            elif SysState == 4: 
                P03 = sym.Matrix([[0],[0],[0],[0],[1],[0]])
                Sflag = 0
            else:
                P_Fail = 1; 
                MTTF = 0; 
                

            M3 = sym.Matrix([[-8*L,     0,    0,    0,    0,    0],
                             [ 8*L,  -7*L,    0,    0,    0,    0],
                             [   0,   6*L, -6*L,    0,    0,    0],
                             [   0,     0,  4*L, -5*L,    0,    0],
                             [   0,     0,    0,  2*L, -4*L,    0],
                             [   0,   1*L,  2*L,  3*L,  4*L,    0]])

            P3 = sym.exp(M3*t)*P03
            P_Fail = P3[-1]

            N3 = sym.Matrix([[-8*L,     0,    0,    0,    0],
                             [ 8*L,  -7*L,    0,    0,    0],
                             [   0,   6*L, -6*L,    0,    0],
                             [   0,     0,  4*L, -5*L,    0],
                             [   0,     0,    0,  2*L, -4*L]])
            
            tt = -1*N3.inv()
            MTTF = sum(tt[Sflag,:])

        else:
            print('The current Motors_Configuration is not defined in this version, please check the updated versions')

        return P_Fail.evalf(subs={L: Motors_Lambda, t: time}), MTTF.evalf(subs={L: Motors_Lambda, t: time})
        
    # def Communication_Failure_Risk_Calc(self.CommStatus, self.Comm_Lambda, self.time)

    #     To be completed...

    def Battery_Failure_Risk_Calc(self, BatteryLevel, time, Lambda = 0.001, alpha = 0.008, beta = 0.007, Battery_degradation_rate = 0.0064):

        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # Program Name : Markov-based Drone's Reliability and MTTF Estimator      %
        # Author       : Koorosh Aslansefat                                       %
        # Version      : 1.0.2                                                    %
        # Description  : A Markov Process-Based Approach for Reliability          %
        #                Evaluation of the Battery System for Drones              %
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        '''
        Provided Markov model is from the following paper:
        
        Kabir, S., Aslansefat, K., Sorokos, I., Papadopoulos, Y., & Gheraibia, Y. (2019, October). 
        A conceptual framework to incorporate complex basic events in HiP-HOPS. 
        In International Symposium on Model-Based Safety and Assessment (pp. 109-124). Springer, Cham.
        
        BatteryLevel: is an integer between 0 and 100. 0 means no charge and 100 means fully charged. 
        
        Lambda: Failure Rate of the Battery System including Battery, voltage regulator and voltage/current meter.
        
        alpha and beta: Are charge and discharge rates.
        '''
        
        import numpy as np  # linear algebra
        import pandas as pd # data processing, CSV file I/O (e.g. pd.read_csv)
        import sympy as sym # Symbolic Calculation

        t = sym.Symbol('t')

        a = alpha
        b = beta
        d = Battery_degradation_rate
        L = Lambda
        
        if BatteryLevel <= 25:
            BatteryStatus = 0
        elif BatteryLevel > 25 & BatteryLevel <= 50:
            BatteryStatus = 1
        elif BatteryLevel > 50 & BatteryLevel <= 75:        
            BatteryStatus = 2
        elif BatteryLevel > 75 & BatteryLevel <= 100:
            BatteryStatus = 3
            
        if BatteryStatus == 3:
            P0_Battery = sym.Matrix([[1],[0],[0],[0],[0],[0],[0],[0]])
            Sflag = 5
        elif BatteryStatus == 2:
            P0_Battery = sym.Matrix([[0],[0],[1],[0],[0],[0],[0],[0]])
            Sflag = 3
        elif BatteryStatus == 1:
            P0_Battery = sym.Matrix([[0],[0],[0],[0],[1],[0],[0],[0]])
            Sflag = 1
        else:
            P_Fail = 1
            MTTF = 0

        M_Battery = sym.Matrix([[-L-a-d,  b,       0,  0,       0,  0, 0, 0],
                                [     a, -b,       0,  0,       0,  0, 0, 0],
                                [     d,  0,  -L-a-d,  b,       0,  0, 0, 0],
                                [     0,  0,       a, -b,       0,  0, 0, 0],
                                [     0,  0,       d,  0,  -L-a-d,  b, 0, 0],
                                [     0,  0,       0,  0,       a, -b, 0, 0],
                                [     0,  0,       0,  0,       d,  0, 0, 0],
                                [     L,  0,       L,  0,       L,  0, 0, 0]])  

        P_Battery = sym.exp(M_Battery*t)*P0_Battery

        P_Battery_Fail = P_Battery[-1] + P_Battery[-2]

        N_Battery = sym.Matrix([[-L-a-d,  b,       0,  0,       0,  0],
                                [     a, -b,       0,  0,       0,  0],
                                [     d,  0,  -L-a-d,  b,       0,  0],
                                [     0,  0,       a, -b,       0,  0],
                                [     0,  0,       d,  0,  -L-a-d,  b],
                                [     0,  0,       0,  0,       a, -b]])

        tt = -1*N_Battery.inv()
        MTTF = sum(tt[Sflag,:])

        return P_Battery_Fail.evalf(subs={t: time}), MTTF.evalf(subs={t: time})
    
    
    #Chip MTTF and Pfail Model estimation based on the temperature. The model uses arrhenious function to estimate the MTTF
    def Chip_MTTF_Model(self,MTTFref, Tr, Ta, u,b, time):
        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # Program Name : Chip MTTF estimation model based on temperature          %
        # Author       : Panagiota Nikolaou                                       %
        # Version      : 1.0.0                                                    %
        # Description  : MTTF and Pfail Chip's estimation based on the dynamic    %
        #                temperature and the time. Use of arrhenius equation to   %
        #                incorporate temperature in the MTTf estimation           %
        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # MTTFref: is the reference MTTF in hours
        # Tr: is the reference temperature
        # Ta: is the actual temperature
        # u: is the drone's utilization
        # Ea: activation energy in electron-volts
        # K: Boltzmans constant (8.617E-05)
        # b: beta describes the Weibull failure distribution,
        #             # b<1.0 indicates infant mortality,
        #             # b=1 means random failure, the failure rate remains constant over time
        #             # b>1 indicates wear-out failure.
        ##

        import math
        import sympy as sym  # Symbolic Calculation

        t = sym.Symbol('t') # t is used for the time
        Ea = 0.3
        K = 8.617 * math.exp(-5)


        AF = sym.exp((Ea / K) * ((1 / (Tr)) - (1 / (Ta)))) #arrhenius equation
        #MTTF estimation based on the arrhenius equation that includes temperature
        MTTF = MTTFref / AF
        MTTFchip = MTTF / u
        #estilation of the pfail using Weibull distribution
        P_Fail=1-sym.exp(-t/MTTFchip**b)

        return P_Fail.evalf(subs={t: time}), MTTFchip.evalf(subs={t: time})

        def Drone_Risk_Calc(time):
            
        import numpy as np
            
            P_Fail_Motor, MTTF_Motor = Motor_Failure_Risk_Calc([1,1,1,1,1,1], 'PNPNPN', 0.001, 100)
            
            P_Fail_Battery, MTTF_Battery = Battery_Failure_Risk_Calc(80, 100)
            
            P_Fail_Processor, MTTF_Processor = Chip_MTTF_Model(400, 30, 50, 1, 1,100)
            
            P_Fail_Total = 1 - (1 - P_Fail_Motor) * (1 - P_Fail_Battery) * (1 - P_Fail_Processor) 
            
            MTTF_Total = np.min([MTTF_Motor, MTTF_Battery, MTTF_Processor])
            
            return P_Fail_Total, MTTF_Total
