from Lib.models import *


# time advance
nbIte = 500000
deltaT = 1.e-5

# initial state
omega0 = 0.
theta0 = 0.

# time solver for engine rotation angle
ode1 = ODEintegration(nbIte, deltaT)
ode1.initF(0.)
ode1.initFDot(0.)

# time solver for arm ration angle
odex = ODEintegration(nbIte, deltaT)
odex.initF(theta0)
odex.initFDot(omega0)