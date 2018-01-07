import numpy as np
import sys


class PID():

    def __init__(self, kp, ki, kd, integralLimit, deltaFilter, heartbeatHz):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integralLimit = integralLimit
        self.deltaFilter = deltaFilter
        self.dxFiltered = 0.
        self.heartbeatHz = heartbeatHz
        self.previousX = 0.
        self.errorIntegral = 0.
        self.proportionalTerm = 0.
        self.integralTerm = 0.
        self.derivativeTerm = 0.

    def computeDerivative(self, x):
        """Compute delta between current and previous input signal"""
        self.previousX = x
        return x - self.previousX

    def exponentialFilter(self, dx):
        """Low pass filtering of input signal derivative"""
        dt = 1.0 / self.heartbeatHz
        return (dx * 1.0 / self.deltaFilter + self.dxFiltered * (self.heartbeatHz - 1.0 / self.deltaFilter)) * dt

    def compute(self, x, target):
        """Compute PID output as the sum of the proportional, integral and derivative of (input x - target)"""

        dx = self.computeDerivative(x)
        self.dxFiltered = self.exponentialFilter(dx)

        error = x - target
        self.errorIntegral += error
        self.errorIntegral = min(self.errorIntegral, self.integralLimit)
        self.errorIntegral = max(self.errorIntegral, -self.integralLimit)
        output = -error * self.kp
        output -= self.errorIntegral * self.ki
        output -= self.dxFiltered * self.kd

        self.proportionalTerm = error * self.kp
        self.integralTerm = self.errorIntegral * self.ki
        self.derivativeTerm = self.dxFiltered * self.kd

        return output





class Battery():

    def __init__(self, v0, capa, maxCurrent):
        self.v0 = v0
        self.capa = capa
        self.maxCurrent = maxCurrent

    def compute_V(self):
        """Compute battery voltage"""
        return self.v0


class Engine():

    def __init__(self, ki, kv, resistance, rotorMass, rotorInternalRadius, rotorExternalRadius):
        self.ki = ki
        self.kv = kv
        self.resistance = resistance
        self.rotorMass = rotorMass
        self.rotorInternalRadius = rotorInternalRadius
        self.rotorExternalRadius = rotorExternalRadius
        self.J = self.rotorMass * \
            (0.5 * self.rotorExternalRadius**2 +
             0.5 * self.rotorInternalRadius**2)
        self.omega_eq = 0.

    def compute_equilibrium_omega(self, propeller, Vs):
        """Steady state engine rotation velocity"""
        roots = np.roots(
            [-propeller.kw, -self.ki / (self.resistance * self.kv), self.ki * Vs / self.resistance])
        roots_positive = roots[np.where(roots >= 0)]
        if len(roots_positive) != 1:
            print('could not find a single positive solution for equilibrium omega')
            print('roots are', roots)
            sys.exit(1)
        else:
            self.omega_eq = roots_positive[0]
            print('equilibrium omega at Vs max=',
                  self.omega_eq * 60. / (2 * np.pi), 'rpm')

    def getTorque(self, omega, Vs):
        """Engine torque in engine axis frame"""
        return (self.ki * Vs / self.resistance) - (self.ki / (self.resistance * self.kv)) * omega

    def getJ(self):
        """Inertial torque in engine axis frame"""
        return self.J


class ESC():
    """Model for an electric engine controller"""

    def __init__(self, frequency, maxThrottle):
        self.frequency = frequency
        self.maxThrottle = maxThrottle

    def computeVs(self, throttle, accu):
        """Compute output controller voltage (= engine voltage if no lost in the wires)"""
        return accu.compute_V() * throttle / self.maxThrottle


class Propeller():
    """Model for a small scale (RC plane) propeller"""

    def __init__(self, diameter, meanChord, bladeMass, nbIte):
        self.cl = 1.23
        self.cd	 = 1.23
        self.diameter = diameter
        self.meanChord = meanChord
        self.bladeMass = bladeMass
        self.kw = np.nan
        self.J = 2 * 2. / 3 * self.bladeMass * (0.5 * diameter)**2

    def initKw(self, rho):
        """Initialize propeller torque coefficient"""
        self.kw = 2 * 0.25 * rho * self.cd	 * \
            (0.5 * self.diameter)**4 * self.meanChord

    def calibrateCl(self, thrust, omega):
        """If you know propeller thrust at a given rotation velocity,
        you can calibrate the cl coefficient instead of using the default value"""
        self.cl = np.sqrt(
            thrust / (0.33 * rho * self.cl * self.meanChord * (0.5 * self.diameter)**3))

    def getThrust(self, rho, omega):
        """Propeller thrust in propeller frame (z = propeller axis)"""
        thrust = 0.33 * rho * self.cl * self.meanChord * \
            omega**2 * (0.5 * self.diameter)**3
        return np.array([0., 0., thrust])

    def getTorque(self, omega):
        """Propeller torque along propeller axis"""
        return -self.kw * omega**2

    def getJ(self):
        """Propeller inertial torque in propeller axis frame"""
        return self.J


class Point_mass():
    """Model for a point mass in rotation around (0,0,0)"""

    def __init__(self, mass, xG, yG, zG):
        self.mass = mass
        self.xG = xG
        self.yG = yG
        self.zG = zG
        self.force = np.zeros((3))

    def computeJ(self, rotAxis):
        """Point mass inertial torque"""
        return self.mass * np.linalg.norm(np.cross(rotAxis, np.array([self.xG, self.yG, self.zG])))

    def setForce(self, force):
        """Update force (3 element array or tuple) acting on the Point mass"""
        self.force = force

    def computeTorque(self, rotAxis):
        """Compute Point mass torque around rotAxis (3 element array or tuple) at (0,0,0)"""
        torqueVector = np.cross(
            np.array([self.xG, self.yG, self.zG]), self.force)
        return np.dot(rotAxis, torqueVector)


class ODEintegration():
    """Time integration of a first or second order ODE"""

    def __init__(self, nbIte, dt):
        self.nbIte = nbIte
        self.dt = dt
        self.f = np.zeros((nbIte))
        self.fDot = np.zeros((nbIte))
        self.fDotDot = np.zeros((nbIte))

    def initF(self, f0):
        """Initial solution for ODE solution f"""
        self.f[:] = f0

    def initFDot(self, fDot0):
        """Initial solution for the derivative of ODE solution f.
        Only necessary for second order ODE"""
        self.fDot[:] = fDot0

    def advanceInTimeFirstOrder(self, ite, fDot):
        """Integration of first order ODE"""
        self.fDot[ite] = fDot
        self.f[ite + 1] = self.fDot[ite] * self.dt + self.f[ite]

    def advanceInTimeSecondOrder(self, ite, fDotDot):
        """Integration of second order ODE"""
        self.fDotDot[ite] = fDotDot
        self.f[ite + 1] = self.fDotDot[ite] * \
            self.dt**2 + 2 * self.f[ite] - self.f[ite - 1]
        self.fDot[ite + 1] = (self.f[ite + 1] - self.f[ite]) / self.dt

    def getTimeAxis(self):
        return np.arange(0., self.nbIte) * self.dt

    def getF(self):
        return self.f

    def getFDot(self):
        return self.fDot


class Meca_model():
    """Mecanical model consisting of N point masses in rotation around (0,0,0)"""

    def __init__(self):
        self.model = []

    def add(self, pointMass):
        """add a point mass to the model"""
        self.model.append(pointMass)

    def computeTotalJ(self, rot_axis):
        """Compute inertial torque of the model"""
        J = 0.
        for m in self.model:
            J += m.computeJ(rot_axis)
        return J

    def computeTotalTorque(self, rotAxis):
        """Compute torque of the model around rotAxis at (0,0,0)"""
        torque = 0.
        for m in self.model:
            torque += m.computeTorque(rotAxis)
        return torque
