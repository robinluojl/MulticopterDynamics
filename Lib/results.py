import sys
import initController as cntrl
import initPID as pid
import initPropulsion as prop
import initAmbient as amb
import initSolver as solver
import initMeca as meca


class Result():

	def __init__(self, name, label, adim):
		self.name = name
		self.label = label
		self.adim = adim


class Result_xy(Result):

	def __init__(self, nbIte, name, label, adim):
		Result.__init__(self, name, label, adim)
		self.nbIte = nbIte
		self.x = 0.
		self.y = 0.

	def update(self, ite):

		if self.name == 'theta':
			self.x = solver.odex.timeAxis[ite]
			self.y = solver.odex.getF()
		elif self.name == 'thetaDot':
			self.x = solver.odex.timeAxis[ite]
			self.y = solver.odex.fDot
		elif self.name == 'Vs':
			self.x = solver.odex.timeAxis[ite]
			self.y = prop.esc.vs
		elif self.name == 'PID theta P':
			self.x = solver.odex.timeAxis[ite]
			self.y = pid.pidTheta.proportionalTerm
		elif self.name == 'PID theta I':
			self.x = solver.ode1.timeAxis[ite]
			self.y = pid.pidTheta.integralTerm	
		elif self.name == 'PID theta D':
			self.x = solver.ode1.timeAxis[ite]
			self.y = pid.pidTheta.derivativeTerm	
		elif self.name == 'PID omega P':
			self.x = solver.ode1.timeAxis[ite]
			self.y = pid.pidOmega.proportionalTerm
		elif self.name == 'PID omega I':
			self.x = solver.ode1.timeAxis[ite]
			self.y = pid.pidOmega.integralTerm	
		elif self.name == 'PID omega D':
			self.x = solver.ode1.timeAxis[ite]
			self.y = pid.pidOmega.derivativeTerm
		else:
			print ('Could not associate result name', self.name, 'with a model data')
			sys.exit(1)