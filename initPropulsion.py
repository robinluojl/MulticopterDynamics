from Lib.models import *
from initAmbient import *
from initController import *


# propulsion args
maxThrust = 11.  # in N


# propeller
d = {'diameter': 10. * 0.0254,
'meanChord': 0.013,
'bladeMass': 0.010,
}

propeller = Propeller(d)
propeller.initKw(rho)
# propeller.calibrateCl(11, 10000 * 2 * np.pi / 60)


# battery
batteryMass = 0.18
V0 = 13.
capa = 2400.
maxCurrent = 20 * 2.4

battery = Battery(V0, capa, maxCurrent)


# ESC
escFreq = 40000

esc = ESC(escFreq, throttleRange)


# engine
omegaMax = 1100 * V0 * 2 * np.pi / 60

d = {'ki': 5.3 * propeller.getMomentAeroCoef(rho) * omegaMax**2 / 31,
'kv': 1100. * 2 * np.pi / 60,
'resistance': 0.5,
'rotorMass': 0.05,
'mass': 0.07,
'rotorInternalRadius': 0.015,
'rotorExternalRadius': 0.02,
}

engine = Engine(d)
