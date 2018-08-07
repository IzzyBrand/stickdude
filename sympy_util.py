import numpy as np
from sympy import lambdify, Dummy

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
    
    def step(self, x, t, values=None):
        # use the input values if provided, else use the stored values
        v = self.values if values is None else values
        assert len(v) == len(self.params)
        
        # the returned values must be a flat array for odient
        args = np.hstack([x, v])

        return np.ravel(np.array(self._rhs_func(*args)))