import unittest
import numpy as np

from SafeDrones.core.SafeDrones import SafeDrones

def test_mfrc():

    inputs = [[1,1,1,1,1,1], [0,1,1,1,1,1], [0,1,1,0,1,1], [1,1,1,1,1,1], \
        [1,1,1,1,1,1]]
    configs = ['PNPNPN', 'PNPNPN', 'PNPNPN', 'PPNNPN', 'PPNNPPNN']
    Motors_Lambda = 0.001
    time = 100
    
    TRUE_LABELS_p_fail  = [0.0489641066173940, 0.195392392995276, 0.329679953964361,\
        0.196030818613950, 0.196030818613950]
    TRUE_LABELS_MTTF = [483.333333333333, 450.000000000000, 350.000000000000,\
        416.666666666667, 452.380952380952]
        
    THRESHOLD = 0.01
    
    evaluator = SafeDrones()
    
    for i in range(5):
        p_fail, mttf = evaluator.Motor_Failure_Risk_Calc(inputs[i], configs[i], Motors_Lambda, time)
        if (abs(p_fail - TRUE_LABELS_p_fail[i]) > THRESHOLD) or (abs(mttf - TRUE_LABELS_MTTF[i]) > THRESHOLD):
            print("MFRC Test Failed for test input: ", i)
            return False
    print("MFRC tests passed.")
    return True
    
def test_bfrc():
    
    input_BatteryStatus = 3
    input_time = 100
    
    TRUE_LABELS_p_fail = 0.0864154183556407
    TRUE_LABELS_MTTF = 949.916941881880
    
    THRESHOLD = 0.01
    
    eval = SafeDrones()
    
    P_Fail, MTTF = eval.Battery_Failure_Risk_Calc(input_BatteryStatus, input_time)
    
    if (abs(P_Fail - TRUE_LABELS_p_fail) > THRESHOLD) or (abs(MTTF - TRUE_LABELS_MTTF) > THRESHOLD):
        print("BFRC Test Failed for test input: ", input_BatteryStatus)
        return False
    print("BFRC tests passed.")
    
    return True

def test_cfrc():
    
    MTTFref_in = 400
    Tr = 51
    Ta = 65
    u = 1
    b = 1
    time_in = 100
    
    THRESHOLD = 0.01
    
    TRUE_LABELS_p_fail = 0.225482713742509
    TRUE_LABELS_MTTF = 391.365996284893
    
    eval = SafeDrones()
    
    P_Fail, MTTF= eval.Chip_MTTF_Model(MTTFref_in, Tr, Ta, u, b, time_in)
    
    if (abs(P_Fail - TRUE_LABELS_p_fail) > THRESHOLD) or (abs(MTTF - TRUE_LABELS_MTTF) > THRESHOLD):
        print("CFRC Test Failed for test input: ", MTTFref_in, Tr, Ta, u, b, time_in)
        return False
    print("CFRC tests passed.")
    
    return True


class TestSafeDroneMethods(unittest.TestCase):
    def testing_motor_failure_risk(self):
        """
        Tests the motor failure risk calculation. 
        """
        print("Testing Motor Failure Risk Calculation . . . . .")
        self.assertTrue(test_mfrc())

    def testing_battery_failure_risk(self):
        """
        Testing battery failure risk calculation.
        """
        print("Testing Battery Failure Risk Calc . . . . .")
        self.assertTrue(test_bfrc())
    
    def testing_chip_failure_risk(self):
        """
        Testing chip failure risk prediction.
        """
        print("Testing chip Failure Risk . . . . .")
        self.assertTrue(test_cfrc())

if __name__ == '__main__':
    unittest.main()

