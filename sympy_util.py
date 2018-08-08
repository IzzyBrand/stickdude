import numpy as np
from sympy import lambdify, Dummy
from sympy.physics.mechanics import msubs

class RHSWrapper():
    def __init__(self, rhs, dynamics, params=[], values=[]):
        self.rhs = rhs
        self.dynamics = dynamics
        self.params = params
        self.values = values
        
        # substitute dummy variables into the equations of motion for later evaluation
        self._dummys = [Dummy() for i in self.dynamics]
        self._dummy_dict = dict(zip(self.dynamics, self._dummys))
        self._dummy_rhs = msubs(self.rhs, self._dummy_dict)
        
        # create a function to evaluate the right hand side
        self._rhs_func = lambdify(list(self._dummys) + list(self.params), self._dummy_rhs)
    
    def __call__(self, x, t, command=None):
        v = self.values

        # if provided, substitue the command into the values list.
        # this assumes that the controlled parameters are first in the list
        if command is not None:
            v[:command.shape[0]] = command

        # the returned values must be a flat array for odient
        args = np.hstack([x, v])

        return np.ravel(np.array(self._rhs_func(*args)))