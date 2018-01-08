import sys
#sys.path.append("Lib")
from Lib.models import *
import matplotlib.pyplot as plt
import Lib.plotting as tool
import time


def run_dynamic_model():
    """Simulation of an arm articulated around x-axis, with an engine and a propeller
    at the end. Engine throttle is controlled by two PID controllers taking as input
    the arm angle and arm velocity. The arm starts from horizontal position.
    The objective is to stabilize the arm horizontally"""

    # air
    rho = 1.2

    # PID control loop frequency in Hz
    heartbeatHz = 160  

    # Arm tilt PID control gains
    tiltKi = 0.25
    tiltKp = 0.5
    tiltErrorIntegralLimit = 2000

    # Arm tilt rate PID control gains
    tiltRateKp = 0.22
    tiltRateKd = 0.5
    tiltRateErrorIntegralLimit = 2000
    tiltRateFilterWidth = 1. / 160

    # throttle order range
    throttleRange = 2000

    # frame dimensions
    armLength = 0.5

    # propulsion args
    maxThrust = 11.  # in N

    # constant throttle order. Throttle control will be throttleOffset + PID control
    # throttleOffset should range between 0 and throttleRange
    throttleOffset = 550.

    # propeller
    Cd = 1.23
    bladeRadius = 0.5 * 10. * 0.0254
    meanChord = 0.013
    momentAeroCoef = 2 * 0.25 * rho * Cd * bladeRadius**4 * meanChord
    bladeMass = 0.010
    propellerMass = 2 * bladeMass

    # battery
    batteryMass = 0.18
    V0 = 13.
    capa = 2400.
    maxCurrent = 20 * 2.4

    # ESC
    escFreq = 40000

    # engine
    engineMass = 0.07
    rotorMass = 0.050
    rotorInternalRadius = 0.015
    rotorExternalRadius = 0.02
    kv = 1100. * 2 * np.pi / 60
    omegaMax = 1100 * V0 * 2 * np.pi / 60
    ki = 5.3*momentAeroCoef * omegaMax**2 / 31
    resistance = 0.5

    # time advance
    nbIte = 500000
    deltaT = 1.e-5

    # initial state
    omega0 = 0.
    theta0 = 0.

    # target state
    targetTheta = 0.

    # plot args
    fontsize = 12

    # initialize objects

    pidTheta = PID(tiltKp, tiltKi, 0., tiltErrorIntegralLimit, 1., heartbeatHz)
    pidOmega = PID(tiltRateKp, 0., tiltRateKd, tiltRateErrorIntegralLimit, tiltRateFilterWidth, heartbeatHz)

    esc = ESC(escFreq, throttleRange)
    battery = Battery(V0, capa, maxCurrent)
    engine = Engine(ki, kv, resistance, rotorMass,
                    rotorInternalRadius, rotorExternalRadius)
    propeller = Propeller(2 * bladeRadius, meanChord, bladeMass, nbIte)
    propeller.initKw(rho)

    # construct mecanical model
    # an engine with propeller at the end of an arm aligned with y and rotating around x
    #      z
    #     ^
    #     |   --|-- propeller
    #     o-----M engine1
    #     x     y

    # Engine + propeller is modelled as a point mass.
    # We add an arbitrary mass that the engine is capable of lifting
    arbitraryMass = 0.18
    pmM1 = Point_mass(engineMass + propellerMass + arbitraryMass, 0., 0.5 * armLength, 0.)

    quadri = Meca_model()
    quadri.add(pmM1)

    # rotation axis
    x = (1, 0, 0)

    # mecanical model inertia and torque
    jx = quadri.computeTotalJ(x)
    mx = quadri.computeTotalTorque(x)
    print ("inertial x momentum", jx)
    print ("total x torque", mx)

    # time solver for engine rotation angle
    ode1 = ODEintegration(nbIte, deltaT)
    ode1.initF(0.)
    ode1.initFDot(0.)

    # time solver for arm ration angle
    odex = ODEintegration(nbIte, deltaT)
    odex.initF(theta0)
    odex.initFDot(omega0)

    # record quantities for post-processing
    outputs = {}
    outputs['vs1'] = np.empty((nbIte))
    outputs['thrust1'] = np.empty((nbIte))
    outputs['pidPTheta'] = np.empty((nbIte))
    outputs['pidITheta'] = np.empty((nbIte))
    outputs['pidDTheta'] = np.empty((nbIte))
    outputs['pidPOmega'] = np.empty((nbIte))
    outputs['pidIOmega'] = np.empty((nbIte))
    outputs['pidDOmega'] = np.empty((nbIte))
    outputs['thetaControl'] = np.empty((nbIte))


    time0 = time.time()

    # time loop

    jProp = propeller.getJ() + engine.getJ()
    vOffset = throttleOffset * V0 / throttleRange
    vs1 = vOffset
    thetaControl = 0.

    # Time loop
    for ite in range(1, nbIte - 1):

        # Dynamic response

        engineTorque1 = propeller.getTorque(
            ode1.fDot[ite]) + engine.getTorque(ode1.fDot[ite], vs1)
        ode1.advanceInTimeSecondOrder(ite, engineTorque1 / jProp)

        # gravty force acting on the point mass
        gravityForce1 = np.array([0.,
            -pmM1.mass * 9.81 * np.sin(odex.f[ite]),
            -pmM1.mass * 9.81 * np.cos(odex.f[ite])])

        # force acting on the force mass is propeller thrust + gravity force
        pmM1.setForce(propeller.getThrust(rho, ode1.fDot[ite]) + gravityForce1)
        totalTorque = quadri.computeTotalTorque(x)
        mx = totalTorque
        odex.advanceInTimeSecondOrder(ite, mx / jx)

        # PID retroaction on arm angle. Retroaction frequency is heartbeatHz

        if (ite % (int)((1. / heartbeatHz) / deltaT) == 0):

            print ('retroaction time', ite * deltaT)

            # PID control on arm rotation velocity
            targetOmega = pidTheta.compute(odex.f[ite], targetTheta)

            # PID control on arm angle
            targetOmegaDot = pidOmega.compute(odex.fDot[ite], targetOmega)
            thetaControl = targetOmegaDot

            # engine controller voltage
            vs1 = esc.computeVs(throttleOffset + thetaControl, battery)

        # update outputs
        outputs['thrust1'][ite] = np.linalg.norm(propeller.getThrust(rho, ode1.fDot[ite]))
        outputs['vs1'][ite] = vs1
        outputs['pidPTheta'][ite] = pidTheta.proportionalTerm
        outputs['pidITheta'][ite] = pidTheta.integralTerm
        outputs['pidDTheta'][ite] = pidTheta.derivativeTerm
        outputs['pidPOmega'][ite] = pidTheta.proportionalTerm
        outputs['pidIOmega'][ite] = pidTheta.integralTerm
        outputs['pidDOmega'][ite] = pidTheta.derivativeTerm
        outputs['thetaControl'][ite] = thetaControl

    print(time.time() - time0)


    # Output plotting

    fig, (ax1) = plt.subplots(1, 1, sharex=True)

    xlabel = ''
    ylabel = ''

    fig.subplots_adjust(hspace=0.5)

    ax1.set_title('', fontweight='bold', fontsize=fontsize)

    ax1.plot(ode1.getTimeAxis(),  outputs['vs1'] / V0,
             'r-', label='Vs')
    ax1.plot(ode1.getTimeAxis(), outputs['thrust1'] / maxThrust,
             'b-', label='propeller thrust in N')
    ax1.plot(ode1.getTimeAxis(), outputs['pidPTheta'],
             'g-', label='pid theta p')
    ax1.plot(ode1.getTimeAxis(), outputs['pidITheta'],
             'g-', label='pid theta i')
    ax1.plot(ode1.getTimeAxis(), outputs['pidDTheta'],
             'g--', label='pid theta d')
    ax1.plot(ode1.getTimeAxis(), outputs['pidPOmega'],
             'g:', label='pid omega p')
    ax1.plot(ode1.getTimeAxis(), outputs['pidIOmega'],
             'y-', label='pid omega i')
    ax1.plot(ode1.getTimeAxis(), outputs['pidDOmega'],
             'y--', label='pid omega d')
    ax1.plot(ode1.getTimeAxis(), outputs['thetaControl'],
             'y:', label='theta control')
    ax1.plot(odex.getTimeAxis(), odex.getFDot(), 'k--', label='rotation velocity around x (rad/s)')
    ax1.plot(odex.getTimeAxis(), odex.getF() * 180 / np.pi, 'k-', label='angle around x (deg)')

    tool.finalize_plot(fig, ax1, None, None, None, None, xlabel, ylabel, fontsize, export_dir='', output_file='',
                       show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, 
                       show=True, tick_fontsize=None)

if __name__ == '__main__':

    run_dynamic_model()
