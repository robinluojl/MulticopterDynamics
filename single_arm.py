import sys
#sys.path.append("Lib")
from Lib.models import *
import matplotlib.pyplot as plt
import Lib.plotting as tool
import time
from initController import *
from initPID import *
from initPropulsion import *
from initAmbient import *
from initSolver import *
from initMeca import *


# constant throttle order. Throttle control will be throttleOffset + PID control
# throttleOffset should range between 0 and throttleRange
throttleOffset = 550.

# target state
targetTheta = 0.

# plot args
fontsize = 12

# mecanical model inertia and torque
jx = quadri.computeTotalJ(x)
mx = quadri.computeTotalTorque(x)
print ("inertial x momentum", jx)
print ("total x torque", mx)

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



def compute_dynamic_response(ite, vs1):

    engineTorque1 = propeller.getTorque(
        ode1.fDot[ite]) + engine.getTorque(ode1.fDot[ite], vs1)
    ode1.advanceInTimeSecondOrder(ite, engineTorque1 / (propeller.getJ() + engine.getJ()))

    # gravty force acting on the point mass
    gravityForce1 = np.array([0.,
        -pmM1.mass * 9.81 * np.sin(odex.f[ite]),
        -pmM1.mass * 9.81 * np.cos(odex.f[ite])])

    # force acting on the force mass is propeller thrust + gravity force
    pmM1.setForce(propeller.getThrust(rho, ode1.fDot[ite]) + gravityForce1)
    totalTorque = quadri.computeTotalTorque(x)
    mx = totalTorque
    odex.advanceInTimeSecondOrder(ite, mx / jx)


def compute_pid_control(ite):

    print ('retroaction time', ite * deltaT)

    # PID control on arm rotation velocity
    targetOmega = pidTheta.compute(odex.f[ite], targetTheta)

    # PID control on arm angle
    targetOmegaDot = pidOmega.compute(odex.fDot[ite], targetOmega)
    thetaControl = targetOmegaDot

    # engine controller voltage
    vs1 = esc.computeVs(throttleOffset + thetaControl, battery)


def post(outputs):

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


def run_dynamic_model():
    """Simulation of an arm articulated around x-axis, with an engine and a propeller
    at the end. Engine throttle is controlled by two PID controllers taking as input
    the arm angle and arm velocity. The arm starts from horizontal position.
    The objective is to stabilize the arm horizontally"""

    vOffset = throttleOffset * V0 / throttleRange
    vs1 = vOffset
    thetaControl = 0.

    time0 = time.time()

    # Time loop
    for ite in range(1, nbIte - 1):

        # Dynamic response
        compute_dynamic_response(ite, vs1)

        # PID retroaction on arm angle. Retroaction frequency is heartbeatHz
        if (ite % (int)((1. / heartbeatHz) / deltaT) == 0):
            compute_pid_control(ite)

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

    post(outputs)


if __name__ == '__main__':

    run_dynamic_model()
