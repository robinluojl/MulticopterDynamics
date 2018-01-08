from Lib.models import *
from initController import *


# Arm tilt PID control gains
d = {'kp': 0.5, 
'ki': 0.25,
'kd': 0., 
'integralLimit': 2000, 
'deltaFilter': 1.,
'heartbeatHz': heartbeatHz}

pidTheta = PID(d)

# Arm tilt rate PID control gains
d = {'kp': 0.22, 
'ki': 0.,
'kd': 0.5, 
'integralLimit': 2000, 
'deltaFilter': 1. / heartbeatHz,
'heartbeatHz': heartbeatHz}

pidOmega = PID(d)