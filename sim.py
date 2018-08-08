from controllers import PID
from dynamics_deriver import f
import numpy as np

pid = PID(1, 0, 0)

def err(state, set_point):
	return set_point[0] - state[0]

# Try to keep the stickdude balanced with the arms out to the side
#
#             ------o-------
#             	    |
#             	    |
#             	    |
#             	    |

sp = [0.0, np.pi/2, -np.pi/2] # set point

# state: x = [theta_0, theta_1, theta_2, theta_0_dot, theta_1_dot, theta_2_dot]
# initial condition is at the set point with zero velocity
x = sp + [0, 0, 0]

t = 0
dt = 0.01 # run at 100hz

while True:
	u = pid.step(x[0], sp[0], dt) # calculate the commanded torques
	x_dot = f(x, t, np.array([u, -u])) # run the system dynamics with the command
	x += x_dot * dt # step the system forward
	t += dt # and time too
	print t, x[:3], u