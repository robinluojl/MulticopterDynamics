import sys
import numpy as np
#sys.path.append("Lib")
import Lib.models as mod
import matplotlib.pyplot as plt
import time
import initController as cntrl
import initPID as pid
import initPropulsion as prop
import initAmbient as amb
import initSolver as solver
import initMeca as meca
import Lib.post as p
import Lib.results as r


# constant throttle order. Throttle control will be throttleOffset + PID control
# throttleOffset should range between 0 and throttleRange
throttleOffset = 550.

# target state
targetTheta = 0.

# mecanical model inertia and torque
jx = meca.quadri.computeTotalJ(meca.axe)
mx = meca.quadri.computeTotalTorque(meca.axe)
print ("inertial x momentum", jx)
print ("total x torque", mx)

# define results
results = []
results.append(r.Result_xy(solver.nbIte - 2, 'theta', 'theta (rad/Pi)', np.pi))
results.append(r.Result_xy(solver.nbIte - 2, 'thetaDot', 'thetaDot ((rad/s) / 2*Pi)', 2 * np.pi))
results.append(r.Result_xy(solver.nbIte - 2, 'PID theta P', 'PID theta P', cntrl.throttleRange))
results.append(r.Result_xy(solver.nbIte - 2, 'PID theta I', 'PID theta I', cntrl.throttleRange))
results.append(r.Result_xy(solver.nbIte - 2, 'PID theta D', 'PID theta D', cntrl.throttleRange))
results.append(r.Result_xy(solver.nbIte - 2, 'PID omega P', 'PID omega P', cntrl.throttleRange))
results.append(r.Result_xy(solver.nbIte - 2, 'PID omega I', 'PID omega I', cntrl.throttleRange))
results.append(r.Result_xy(solver.nbIte - 2, 'PID omega D', 'PID omega D', cntrl.throttleRange))
results.append(r.Result_xy(solver.nbIte - 2, 'Vs', 'Vs/V0', prop.battery.v0))

fontsize = 12
nbSubplot = 4
styles = ['k-', 'k--', 'b-', 'b--', 'b:', 'g-', 'g--', 'g:', 'y-']
subPlotIndex = [0, 0, 1, 1, 1, 2, 2, 2, 3]
post = p.Post(fontsize)
post.createXYPlot(nbSubplot, styles, subPlotIndex, results)


def compute_dynamic_response(ite, throttleControl):
    """Computes the forces and torques acting on the mechanical model.
    The torques are the sum of gravity and propoller thrust. Then
    the solver advances in time and computes the new rotation velocity
    and angle of the mechanical model"""

    prop.esc.computeVs(throttleControl, prop.battery)
    thetaDot1 = solver.ode1.fDot
    theta1 = solver.ode1.getF()
    thetaDot = solver.odex.fDot
    theta = solver.odex.getF()

    engineTorque1 = prop.propeller.getTorque(
        thetaDot1) + prop.engine.getTorque(thetaDot1, prop.esc.vs)
    solver.ode1.advanceInTimeSecondOrder(ite, engineTorque1 / (prop.propeller.getJ() + prop.engine.getJ()))

    # update multicopter angle
    meca.quadri.theta = solver.odex.getF()

    # gravty force acting on the point mass
    meca.quadri.updateGravityForce()

    # force acting on the force mass is propeller thrust + gravity force
    meca.pmM1.force = prop.propeller.getThrust(amb.rho, thetaDot1)
    totalTorque = meca.quadri.computeTotalTorque(meca.axe)
    mx = totalTorque
    solver.odex.advanceInTimeSecondOrder(ite, mx / jx)



def compute_pid_control(ite):
    """Computes the control order from the PID controllers, based on the new mechanical
    angle. This function must be called at the loop frequency of the multicopter controller"""

    print ('retroaction time', ite * solver.deltaT)

    thetaDot = solver.odex.fDot
    theta = solver.odex.getF()

    # PID control on arm rotation velocity
    targetOmega = pid.pidTheta.compute(theta, targetTheta)

    # PID control on arm angle
    targetOmegaDot = pid.pidOmega.compute(thetaDot, targetOmega)
    thetaControl = targetOmegaDot

    return throttleOffset + thetaControl



def run_dynamic_model():
    """Simulation of an arm articulated around x-axis, with an engine and a propeller
    at the end. Engine throttle is controlled by two PID controllers taking as input
    the arm angle and arm velocity. The arm starts from horizontal position.
    The objective is to stabilize the arm horizontally"""

    vOffset = throttleOffset * prop.battery.v0 / cntrl.throttleRange
    vs1 = vOffset
    throttleControl = throttleOffset

    time0 = time.time()

    # Time loop
    for ite in range(1, solver.nbIte - 1):

        # Dynamic response
        compute_dynamic_response(ite, throttleControl)

        # PID retroaction on arm angle. Retroaction frequency is heartbeatHz
        if (ite % (int)((1. / cntrl.heartbeatHz) / solver.deltaT) == 0):

            throttleControl = compute_pid_control(ite)

            # update results and post (plotting)
            for r in results:
                r.update(ite - 1)
            post.plt.set()

    print(time.time() - time0)

    post.plt.plot()


if __name__ == '__main__':

    run_dynamic_model()
    # anim = p.animation.FuncAnimation(p.fig, p.animate, init_func=p.init,
    # frames=len(p.circle_colors[0][0]), blit=True)
    # plt.show()