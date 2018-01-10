import sys
#sys.path.append("Lib")
from Lib.models import *
import matplotlib.pyplot as plt
import time
import initController as cntrl
import initPID as pid
import initPropulsion as prop
import initAmbient as amb
import initSolver as solver
import initMeca as meca
import post as p


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



def compute_dynamic_response(ite, throttleControl):
    """Computes the forces and torques acting on the mechanical model.
    The torques are the sum of gravity and propoller thrust. Then
    the solver advances in time and computes the new rotation velocity
    and angle of the mechanical model"""

    vs1 = prop.esc.computeVs(throttleControl, prop.battery)
    thetaDot1 = solver.ode1.fDot[ite]
    theta1 = solver.ode1.f[ite]
    thetaDot = solver.odex.fDot[ite]
    theta = solver.odex.f[ite]

    engineTorque1 = prop.propeller.getTorque(
        thetaDot1) + prop.engine.getTorque(thetaDot1, vs1)
    solver.ode1.advanceInTimeSecondOrder(ite, engineTorque1 / (prop.propeller.getJ() + prop.engine.getJ()))

    # update multicopter angle
    meca.quadri.theta = solver.odex.f[ite]

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

    thetaDot = solver.odex.fDot[ite]
    theta = solver.odex.f[ite]

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

        # update outputs

        p.plt.curves[0].x[ite - 1] = solver.ode1.timeAxis[ite]
        p.plt.curves[0].y[ite - 1] = np.linalg.norm(prop.propeller.getThrust(amb.rho, solver.ode1.fDot[ite]))

        p.plt.curves[1].x[ite - 1] = solver.ode1.timeAxis[ite]
        p.plt.curves[1].y[ite - 1] = vs1

        p.plt.curves[2].x[ite - 1] = solver.ode1.timeAxis[ite]
        p.plt.curves[2].y[ite - 1] = pid.pidTheta.proportionalTerm
        p.plt.curves[3].x[ite - 1] = solver.ode1.timeAxis[ite]
        p.plt.curves[3].y[ite - 1] = pid.pidTheta.integralTerm
        p.plt.curves[4].x[ite - 1] = solver.ode1.timeAxis[ite]
        p.plt.curves[4].y[ite - 1] = pid.pidTheta.derivativeTerm

        p.plt.curves[5].x[ite - 1] = solver.ode1.timeAxis[ite]
        p.plt.curves[5].y[ite - 1] = pid.pidOmega.proportionalTerm
        p.plt.curves[6].x[ite - 1] = solver.ode1.timeAxis[ite]
        p.plt.curves[6].y[ite - 1] = pid.pidOmega.integralTerm
        p.plt.curves[7].x[ite - 1] = solver.ode1.timeAxis[ite]
        p.plt.curves[7].y[ite - 1] = pid.pidOmega.derivativeTerm

        p.plt.curves[8].x[ite - 1] = solver.ode1.timeAxis[ite]
        p.plt.curves[8].y[ite - 1] = throttleControl

        p.plt.curves[9].x[ite - 1] = solver.ode1.timeAxis[ite]
        p.plt.curves[9].y[ite - 1] = prop.esc.computeVs(throttleControl, prop.battery)
        p.plt.curves[10].x[ite - 1] = solver.ode1.timeAxis[ite]
        p.plt.curves[10].y[ite - 1] = solver.odex.getF()[ite]

    print(time.time() - time0)

    p.plt.plot()


if __name__ == '__main__':

    run_dynamic_model()
