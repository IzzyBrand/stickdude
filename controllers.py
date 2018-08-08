import numpy as np

class Controller(object):

	def __init__(self, dt=0.0):
		self.dt = dt

	def step(self, state, set_point, dt=None):
		""" Step the controller. Return commands """
		raise NotImplementedError, 'step'

	def reset(self):
		pass

class PID(Controller):
	""" PID controller. """

	def __init__(self, kp, ki, kd, control_range=None, dt=None):
		# super(PID).__init__(self)
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.range = control_range
		self.dt = dt
		self.reset()

	def step(self, state, set_point, dt=None):
		err = set_point - state				# caluculate the error
		self._int_err += err * dt			# its integral
		d_err = (err - self._prev_err)/dt   # and its derivative

		p = self.kp * err 					# calculate p, i, d components of control
		i = self.ki * self._int_err
		d = self.kd * d_err

		u = p + i + d 						# sum them

		if self.range is not None:
			u = np.clip(u, self.range[0], self.range[1]) # threshold output
			
		return u

	def reset(self):
		self._prev_err = 0
		self._int_err = 0
