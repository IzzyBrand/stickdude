import numpy as np
from sympy import Matrix

class PendulumDrawer():
    def __init__(self, point_pairs, inertial_frame, origin, dynamics, params, values=None):
        """ Tool displaying the state of a sympy pendulum.

        Args:
            point_pairs: (list(tuple(Point, Point))) A list of the point pairs corresponding to the ends of segments
            inertial_frame: (ReferenceFrame) The base reference frame of the system
            origin: (Point) The origin of the system
            dynamics: list(dynamicsymbol): he state of the system which will change from frame-to-frame when rendering
            params: list(symbol) Other system parameters (pendulum length, for example)
            values: list(float) The values of those dynamics symbols
        """
        self.expressions_raw = [self._get_expression_for_point_pair(p, inertial_frame, origin) for p in point_pairs]
        self.dynamics = dynamics
        self.params = params
        self.expressions = None
        if values is not None:
            self.set_values(values)
    
    def _get_coords(self, dynamics_values):
        if self.expressions is None:
            print 'Use PendulumDrawer.set_values() to assign values to the parameter list'
            return []
        
        return np.hstack([msubs(e, dict(zip(self.dynamics, dynamics_values))) for e in self.expressions]).T
        
    def set_values(self, values):
        assert len(values) == len(self.params)
        print self.params
        print values
        self.expressions = [msubs(e, dict(zip(self.params, values))) for e in self.expressions_raw]
        
    def _get_expression_for_point_pair(self, point_pair, inertial_frame, origin):
        return Matrix([self._get_expression_for_point(point_pair[0], inertial_frame, origin),
                       self._get_expression_for_point(point_pair[1], inertial_frame, origin)])
    
    def _get_expression_for_point(self, point, inertial_frame, origin):
        e = point.pos_from(origin).express(inertial_frame).args
        return Matrix([0,0,0]) if len(e) == 0 else e[0][0]
        
    def get_segment_list(self, dynamics_values):
        """ Given the a state, q, returns a list of pairs of points for each segment to be drawn

        Args:
            dynamics_values: (list(float)) the state of the system, q

        Returns:
            An [n x 2 x 2] array, A, where A[i,:,:] = [[y_i, y_i+1], [x_i, x_i+1]]. Each row of the array
            is an yy, xx pair that represents the endpoints of one of the pendulum segments. They are organized
            yy, xx as opposed to xy, xy in order to make plotting with matplotlib easier.

        """
        coords = self._get_coords(dynamics_values)
        xs = coords[:,[0,3]]
        ys = coords[:,[1,4]]
        return np.stack([ys,xs], axis=1)