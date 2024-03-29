import numpy as np
import pandas as pd # data processing, CSV file I/O (e.g. pd.read_csv)
import sympy as sym

class SafeDrones:
    
    def __init__(self):
        
        # Observasions or Symptoms from Diagnosis System
        self.MotorStatus = None
        self.CommStatus = None
        self.BattLvl = None
        self.alpha = None
        self.beta = None
        
        # Drone Settings
        self.Motors_Configuration = None
        
        # Defining Failure Rates
        self.Motors_Lambda = None
        self.Comm_Lambda = None
        self.Batt_Lambda = None
        self.Battery_degradation_rate = None
        self.MTTFref = None
        self.Tr = None
        self.Ta = None
        self.u = None
        self.b = None
        
        # Mission Time
        self.time = None

        self.danger_threshold = None
        self.collision_threshold = None
        self.GPS_Lambda = None
        self.SatStatus = None

        self.Set_Variables()

    def Set_Variables(self, MotorStatus=[1,1,1,1,1,1], Motors_Configuration='PNPNPN',\
            Motors_Lambda = 0.001, Batt_Lambda = 0.001, alpha = 0.008, beta = 0.007, \
            Battery_degradation_rate = 0.0064, BatteryLevel=80, MTTFref=400, \
            Tr=30, Ta=50, u=1,b=1, time=100, danger_threshold=2, collision_threshold=0, \
            GPS_Lambda=0.001, SatStatus = 0):
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # Version      : 1.0.0                                                    %
        # Description  : set variables used in the program                        %
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # Observasions or Symptoms from Diagnosis System
        self.MotorStatus = MotorStatus
        self.CommStatus = None
        self.BattLvl = BatteryLevel
        self.alpha = alpha
        self.beta = beta
        
        # Drone Settings
        self.Motors_Configuration = Motors_Configuration
        
        # Defining Failure Rates
        self.Motors_Lambda = Motors_Lambda
        self.Comm_Lambda = None
        self.Batt_Lambda = Batt_Lambda
        self.Battery_degradation_rate = Battery_degradation_rate
        self.MTTFref = MTTFref
        self.Tr = Tr
        self.Ta = Ta
        self.u = u
        self.b = b
        
        # Mission Time
        self.time = time

        #Collision related
        self.danger_threshold = danger_threshold
        self.collision_threshold = collision_threshold
        self.GPS_Lambda = GPS_Lambda
        self.SatStatus = SatStatus
        

    def Motor_Failure_Risk_Calc(self, MotorStatus=None, Motors_Configuration=None, Motors_Lambda=None, time=None):

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
        
        if MotorStatus is None:
            MotorStatus = self.MotorStatus
        if Motors_Configuration is None:
            Motors_Configuration = self.Motors_Configuration
        if Motors_Lambda is None:
            Motors_Lambda = self.Motors_Lambda
        if time is None:
            time = self.time

        L = sym.Symbol('L')
        t = sym.Symbol('t')
        
        SysState = sum(MotorStatus)

        if  Motors_Configuration == 'PNPN':
            if len(MotorStatus) > 4:
                   print('The MotorStatus vector should match Motors_Configuration vector') 
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

    def Battery_Failure_Risk_Calc_precise(self, BatteryLevel=None, time=None, Lambda = None, alpha = None, beta = None, Battery_degradation_rate = None):

        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # Program Name : Markov-based Drone's Reliability and MTTF Estimator      %
        # Author       : Koorosh Aslansefat                                       %
        # Version      : 1.0.2                                                    %
        # Description  : A Markov Process-Based Approach for Reliability          %
        #                Evaluation of the Battery System for Drones              %
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        '''
        This function computes precise probability of failure. But it takes longer. 

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
        
        if BatteryLevel is None:
            BatteryLevel = self.BattLvl
        if Battery_degradation_rate is None:
            Battery_degradation_rate = self.Battery_degradation_rate 
        if beta is None:
            beta = self.beta
        if alpha is None:
            alpha = self.alpha
        if Lambda is None:
            Lambda = self.Batt_Lambda
        if time is None:
            time = self.time

        t = sym.Symbol('t')

        a = alpha
        b = beta
        d = Battery_degradation_rate
        L = Lambda
        
        if BatteryLevel <= 25:
            BatteryStatus = 0
        elif BatteryLevel > 25 and BatteryLevel <= 50:
            BatteryStatus = 1
        elif BatteryLevel > 50 and BatteryLevel <= 75:        
            BatteryStatus = 2
        elif BatteryLevel > 75 and BatteryLevel <= 100:
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
            
        if BatteryStatus != 0:
        
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
        else:
            return P_Fail, MTTF
        '''
        import numpy as np  # linear algebra
        import pandas as pd # data processing, CSV file I/O (e.g. pd.read_csv)
        import sympy as sym # Symbolic Calculation
        
        if BatteryLevel is None:
            BatteryLevel = self.BattLvl
        if Battery_degradation_rate is None:
            Battery_degradation_rate = self.Battery_degradation_rate 
        if beta is None:
            beta = self.beta
        if alpha is None:
            alpha = self.alpha
        if Lambda is None:
            Lambda = self.Batt_Lambda
        if time is None:
            time = self.time

        t = sym.Symbol('t')

        a = alpha
        b = beta
        d = Battery_degradation_rate
        L = Lambda
        
        if BatteryLevel <= 25:
            BatteryStatus = 0
        elif BatteryLevel > 25 and BatteryLevel <= 50:
            BatteryStatus = 1
        elif BatteryLevel > 50 and BatteryLevel <= 75:        
            BatteryStatus = 2
        elif BatteryLevel > 75 and BatteryLevel <= 100:
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
            
        if BatteryStatus != 0:
        
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
        else:
            return P_Fail, MTTF
            '''
    
    def Battery_Failure_Risk_Calc(self, BatteryLevel=None, time=None, Lambda = None, alpha = None, beta = None, Battery_degradation_rate = None):

        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # Program Name : Markov-based Drone's Reliability and MTTF Estimator      %
        # Author       : Koorosh Aslansefat, Akram                                       %
        # Version      : 1.0.2                                                    %
        # Description  : A Markov Process-Based Approach for Reliability          %
        #                Evaluation of the Battery System for Drones              %
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        '''
        This function computes approximate probability of failure. But it is quick.

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
        from scipy.linalg import expm

        
        if BatteryLevel is None:
            BatteryLevel = self.BattLvl
        if Battery_degradation_rate is None:
            Battery_degradation_rate = self.Battery_degradation_rate 
        if beta is None:
            beta = self.beta
        if alpha is None:
            alpha = self.alpha
        if Lambda is None:
            Lambda = self.Batt_Lambda
        if time is None:
            time = self.time

        t = sym.Symbol('t')

        a = alpha
        b = beta
        d = Battery_degradation_rate
        L = Lambda
        
        if BatteryLevel <= 25:
            BatteryStatus = 0
        elif BatteryLevel > 25 and BatteryLevel <= 50:
            BatteryStatus = 1
        elif BatteryLevel > 50 and BatteryLevel <= 75:        
            BatteryStatus = 2
        elif BatteryLevel > 75 and BatteryLevel <= 100:
            BatteryStatus = 3
        
        if BatteryStatus == 3:
            P0_Battery = np.matrix([[1],[0],[0],[0],[0],[0],[0],[0]])
            Sflag = 5
        elif BatteryStatus == 2:
            P0_Battery = np.matrix([[0],[0],[1],[0],[0],[0],[0],[0]])
            Sflag = 3
        elif BatteryStatus == 1:
            P0_Battery = np.matrix([[0],[0],[0],[0],[1],[0],[0],[0]])
            Sflag = 1
        else:
            P_Fail = 1
            MTTF = 0
            return P_Fail, MTTF
        
        P0_Battery = P0_Battery.astype(np.float64)

        if BatteryStatus != 0:
        
            M_Battery = np.matrix([[-L-a-d,  b,       0,  0,       0,  0, 0, 0],
                                    [     a, -b,       0,  0,       0,  0, 0, 0],
                                    [     d,  0,  -L-a-d,  b,       0,  0, 0, 0],
                                    [     0,  0,       a, -b,       0,  0, 0, 0],
                                    [     0,  0,       d,  0,  -L-a-d,  b, 0, 0],
                                    [     0,  0,       0,  0,       a, -b, 0, 0],
                                    [     0,  0,       0,  0,       d,  0, 0, 0],
                                    [     L,  0,       L,  0,       L,  0, 0, 0]], dtype=np.float64) 
            

            # P_Battery1 = sym.exp(M_Battery*np.float64(t))*P0_Battery
            P_Battery1 = expm(M_Battery*np.float64(time))*P0_Battery


            P_Battery_Fail = P_Battery1[-1] + P_Battery1[-2]

            N_Battery = np.matrix([[-L-a-d,  b,       0,  0,       0,  0],
                                    [     a, -b,       0,  0,       0,  0],
                                    [     d,  0,  -L-a-d,  b,       0,  0],
                                    [     0,  0,       a, -b,       0,  0],
                                    [     0,  0,       d,  0,  -L-a-d,  b],
                                    [     0,  0,       0,  0,       a, -b]], dtype=np.float64) 

            gg = np.linalg.inv(N_Battery)
            tt = -1*gg
            MTTF = np.sum(tt[Sflag,:])

        return P_Battery_Fail[0, 0], MTTF

    #Chip MTTF and Pfail Model estimation based on the temperature. The model uses arrhenious function to estimate the MTTF
    def Chip_MTTF_Model(self,MTTFref=None, Tr=None, Ta = None, u=None,b = None, time = None):
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
        
        if MTTFref is None:
            MTTFref = self.MTTFref
        if Tr is None:
            Tr = self.Tr
        if Ta is None:
            Ta = self.Ta
        if u is None:
            u = self.u
        if b is None:
            b = self.b
        if time is None:
            time = self.time

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

    # def GPS_Failure_Risk_Calc(self, SatStatus=None, time=None, Lambda = None, ):

    #     #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    #     # Program Name : Markov-based Drone's Reliability and MTTF Estimator      %
    #     # Author       : Koorosh Aslansefat                                       %
    #     # Version      : 1.0.0                                                    %
    #     # Description  : A Markov Process-Based Approach for Reliability          %
    #     #                Evaluation of the GPS System for Drones                  %
    #     #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    #     '''
        
    #     SatStatus: is an integer between 0 and 11. 0 means no satellite is available and 14 means fully satellite coverage. 
        
    #     Lambda: Failure Rate of the GPS System.
        
    #     time: Time of the mission
    #     '''
        
    #     import numpy as np  # linear algebra
    #     import pandas as pd # data processing, CSV file I/O (e.g. pd.read_csv)
    #     import sympy as sym # Symbolic Calculation
        
    #     t = sym.Symbol('t')
        
    #     if SatStatus == None:
    #         SatStatus = self.SatStatus
    #     if Lambda == None:
    #         Lambda = self.GPS_Lambda
    #     L = Lambda

    #     if SatStatus == 14:
    #         P0_GPS = sym.Matrix([[1],[0],[0],[0],[0],[0],[0],[0],[0]])
    #         Sflag = 7
    #     elif SatStatus == 13:
    #         P0_GPS = sym.Matrix([[0],[1],[0],[0],[0],[0],[0],[0],[0]])
    #         Sflag = 6
    #     elif SatStatus == 12:
    #         P0_GPS = sym.Matrix([[0],[0],[1],[0],[0],[0],[0],[0],[0]])
    #         Sflag = 5
    #     elif SatStatus == 11:
    #         P0_GPS = sym.Matrix([[0],[0],[0],[1],[0],[0],[0],[0],[0]])
    #         Sflag = 4
    #     elif SatStatus == 10:
    #         P0_GPS = sym.Matrix([[0],[0],[0],[0],[1],[0],[0],[0],[0]])
    #         Sflag = 3
    #     elif SatStatus == 9:
    #         P0_GPS = sym.Matrix([[0],[0],[0],[0],[0],[1],[0],[0],[0]])
    #         Sflag = 2
    #     elif SatStatus == 8:
    #         P0_GPS = sym.Matrix([[0],[0],[0],[0],[0],[0],[1],[0],[0]])
    #         Sflag = 1
    #     elif SatStatus == 7:
    #         P0_GPS = sym.Matrix([[0],[0],[0],[0],[0],[0],[0],[1],[0]])
    #         Sflag = 0
    #     else:
    #         P_Fail = 1
    #         MTTF = 0
    #         return P_Fail, MTTF

    #     M_GPS = sym.Matrix([[-14*L,     0,     0,     0,     0,     0,     0,     0,    0],
    #                         [ 14*L, -13*L,     0,     0,     0,     0,     0,     0,    0],
    #                         [    0,  13*L, -12*L,     0,     0,     0,     0,     0,    0],
    #                         [    0,     0,  12*L, -11*L,     0,     0,     0,     0,    0],
    #                         [    0,     0,     0,  11*L, -10*L,     0,     0,     0,    0],
    #                         [    0,     0,     0,     0,  10*L,  -9*L,     0,     0,    0],
    #                         [    0,     0,     0,     0,     0,   9*L,  -8*L,     0,    0],
    #                         [    0,     0,     0,     0,     0,     0,   8*L,  -7*L,    0],
    #                         [    0,     0,     0,     0,     0,     0,     0,   7*L,    0]])

    #     P_GPS = sym.exp(M_GPS*t)*P0_GPS

    #     P_GPS_Fail = P_GPS[-1]

    #     N_GPS = sym.Matrix([[-14*L,     0,     0,     0,     0,     0,     0,     0],
    #                         [ 14*L, -13*L,     0,     0,     0,     0,     0,     0],
    #                         [    0,  13*L, -12*L,     0,     0,     0,     0,     0],
    #                         [    0,     0,  12*L, -11*L,     0,     0,     0,     0],
    #                         [    0,     0,     0,  11*L, -10*L,     0,     0,     0],
    #                         [    0,     0,     0,     0,  10*L,  -9*L,     0,     0],
    #                         [    0,     0,     0,     0,     0,   9*L,  -8*L,     0],
    #                         [    0,     0,     0,     0,     0,     0,   8*L,  -7*L]])
        
    #     tt = -1*N_GPS.inv()
        
    #     MTTF_GPS = sum(tt[Sflag,:])

    #     return P_GPS_Fail.evalf(subs={t: time}), MTTF_GPS.evalf(subs={t: time})


    # def GPS_Failure_Risk_Calc(self, SatStatus=None, time=None, Lambda = None, MaxSat = 28, MinSat = 7):

    #     #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    #     # Program Name : Markov-based Drone's Reliability and MTTF Estimator      %
    #     # Author       : Koorosh Aslansefat                                       %
    #     # Version      : 1.0.5                                                    %
    #     # Description  : A Markov Process-Based Approach for Reliability          %
    #     #                Evaluation of the GPS System for Drones                  %
    #     #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    #     '''
        
    #     SatStatus: is an integer between 0 and MaxSat. 0 means no satellite is available and 14 means fully satellite coverage. 
        
    #     Lambda: Failure Rate of the GPS System.
        
    #     time: Time of the mission
    #     '''
        
    #     import numpy as np  # linear algebra
    #     import pandas as pd # data processing, CSV file I/O (e.g. pd.read_csv)
    #     import sympy as sym # Symbolic Calculation
        
    #     t = sym.Symbol('t')
    #     L = Lambda
        
    #     if SatStatus > MaxSat:
    #         print("The satellite status can not be more than maximum number of satellites. ")
    #         raise ValueError
    #         return None, None

    #     if MinSat < SatStatus <= MaxSat:
    #         P0_GPS = sym.zeros(MaxSat-MinSat+1, 1)
    #         P0_GPS[SatStatus - MinSat] = 1
    #         Sflag = MaxSat - SatStatus
    #     else:
    #         P_Fail = 1
    #         MTTF = 0
    #         return P_Fail, MTTF
        
    #     M_GPS_updated = sym.zeros(MaxSat-MinSat+1, MaxSat-MinSat+1)
    #     start_value = -1*MaxSat
        
    #     for i in range(MaxSat-MinSat+1):
    #         if i == (MaxSat-MinSat):
    #             M_GPS_updated[i, i] = (start_value + i) * L
    #         else:
    #             M_GPS_updated[i, i] = (start_value + i) * L
    #             M_GPS_updated[i+1, i] = -(start_value + i) * L 
                
    #     M_GPS = sym.Matrix(M_GPS_updated)
        
    #     P_GPS = sym.exp(M_GPS*t)*P0_GPS

    #     P_GPS_Fail = P_GPS[-1]
        
    #     N_GPS = M_GPS[:-1, :-1] 
        
    #     tt = -1*N_GPS.inv()
        
    #     MTTF_GPS = sum(tt[Sflag,:])

    #     return P_GPS_Fail.evalf(subs={t: time}), MTTF_GPS.evalf(subs={t: time})
    
    def calculate_collision_risk(self, uav_1_trajectory, uav_2_trajectory, danger_threshold=-1, collision_threshold=-1):  # self.danger_threshold, self.collision_threshold
        # Check that both trajectories have the same length
        assert len(uav_1_trajectory) == len(uav_2_trajectory), "Trajectories must have the same length"
    
        danger_zone_count = 0
        collision_zone_count = 0

        if danger_threshold != -1:
            self.danger_threshold = danger_threshold
        if collision_threshold != -1:
            self.collision_threshold = collision_threshold
    
        for point_1, point_2 in zip(uav_1_trajectory, uav_2_trajectory):
            distance = np.linalg.norm(np.array(point_1) - np.array(point_2))
            
            if distance < self.collision_threshold:
                collision_zone_count += 1
            if distance < self.danger_threshold:
                danger_zone_count += 1
    
        danger_zone_risk = danger_zone_count / len(uav_1_trajectory)
        collision_zone_risk = collision_zone_count / len(uav_1_trajectory)
    
        return danger_zone_risk, collision_zone_risk

    def GPS_Failure_Risk_Calc(self, SatStatus=None, time=None, Lambda=None, MaxSat=29, MinSat=17):

        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # Program Name : k-out-of-n based Reliability and MTTF Estimator          %
        # Authors      : Koorosh Aslansefat, Mohammed Naveed Akram, Martin Walker %
        # Version      : 1.0.7                                                    %
        # Description  : A k-out-of-n-Based Approach for Reliability              %
        #                Evaluation of the GPS System for Drones                  %
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        '''
        SatStatus: is an integer between 0 and MaxSat. 
        Lambda: Failure Rate of a single GPS satellite/connection.
        time: Time of the mission.
        '''

        import math
        
        if Lambda == None:
            Lambda = self.GPS_Lambda

        p0_fail = 1 - math.exp(-Lambda * time)  # Probability of failure for one satellite connection
        numSats = SatStatus - MinSat + 1        # Number of satellites until system failure
        if numSats <= 0:
            return 1, 0                         # System failure: insufficient satellites
        k = MinSat
        n = SatStatus

        # Calculate overall probability of failure
        p0_success = 1 - p0_fail
        if k < 0 or k > n:                      # Parameter error
            return 1, 0
        
        p_success = 0
        for i in range(k, n+1):                 # n is inclusive
            fac = math.factorial(n) / (math.factorial(i) * math.factorial(n - i))
            p_success += fac * (p0_success ** i) * (p0_fail ** (n-i))
        p_fail = 1 - p_success

        # Calculate MTTF
        mttf = 0
        for i in range(MinSat, SatStatus+1):
            mttf += 1/i
        mttf *= 1/Lambda

        # Return
        return p_fail, mttf

    # def GPS_Failure_Risk_Calc(self, SatStatus=None, time=None, Lambda = None, MaxSat = 29, MinSat = 17):

    #     #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    #     # Program Name : Markov-based Drone's Reliability and MTTF Estimator      %
    #     # Authors      : Koorosh Aslansefat and Mohammed Naveed Akram             %
    #     # Version      : 1.0.6                                                    %
    #     # Description  : A Markov Process-Based Approach for Reliability          %
    #     #                Evaluation of the GPS System for Drones                  %
    #     #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    #     '''
    #     SatStatus: is an integer between 0 and MaxSat. 0 means no satellite is available and 14 means fully satellite coverage. 
    #     Lambda: Failure Rate of the GPS System.
    #     time: Time of the mission
    #     '''
        
    #     t = sym.Symbol('t')

    #     if Lambda == None:
    #         Lambda = self.GPS_Lambda
    #     L = Lambda

    #     if SatStatus is None:
    #         SatStatus = self.SatStatus
        
    #     if MinSat < SatStatus <= MaxSat:
    #         P0_GPS = sym.zeros(MaxSat-MinSat+2, 1) # +2 to account for the additional row
    #         P0_GPS[MaxSat - SatStatus] = 1
    #         Sflag_GPS = (MaxSat-MinSat) - (MaxSat - SatStatus)
    #     else:
    #         P_Fail = 1
    #         MTTF = 0
    #         return P_Fail, MTTF
        
    #     M_GPS_updated = sym.zeros(MaxSat-MinSat+2, MaxSat-MinSat+2) # +2 to account for the additional row and column
    #     start_value = -1*MaxSat
        
    #     for i in range(MaxSat-MinSat+2): # +2 to loop through the additional row
    #         if i == MaxSat-MinSat+1: # +1 to account for the additional row
    #             M_GPS_updated[i, i] = (start_value + i) * L
    #         else:
    #             M_GPS_updated[i, i] = (start_value + i) * L
    #             M_GPS_updated[i+1, i] = -(start_value + i) * L
                
    #     M_GPS_updated[-1,-1] = 0
                
    #     M_GPS = sym.Matrix(M_GPS_updated)
        
    #     P_GPS = sym.exp(M_GPS*t)*P0_GPS

    #     P_GPS_Fail = P_GPS[-1]
        
    #     N_GPS = M_GPS[:-1, :-1] 
        
    #     tt = -1*N_GPS.inv()
        
    #     MTTF_GPS = sum(tt[Sflag_GPS,:])

    #     return P_GPS_Fail.evalf(subs={t: time}), MTTF_GPS.evalf(subs={t: time})

    def Drone_Risk_Calc(self):
            
        import numpy as np
            
        P_Fail_Motor, MTTF_Motor = self.Motor_Failure_Risk_Calc(self.MotorStatus, self.Motors_Configuration, self.Motors_Lambda, self.time)
        
        P_Fail_Battery, MTTF_Battery = self.Battery_Failure_Risk_Calc(self.BattLvl, self.time)
        
        P_Fail_Processor, MTTF_Processor = self.Chip_MTTF_Model(self.MTTFref, self.Tr, self.Ta, self.u, self.b, self.time)
        
        P_Fail_Total = 1 - (1 - P_Fail_Motor) * (1 - P_Fail_Battery) * (1 - P_Fail_Processor) 
        
        MTTF_Total = np.min([MTTF_Motor, MTTF_Battery, MTTF_Processor])
        
        return P_Fail_Total, MTTF_Total
