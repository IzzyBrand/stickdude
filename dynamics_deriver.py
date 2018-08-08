import cPickle as pkl
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.integrate import odeint
from sympy import symbols, lambdify, Dummy, integrate, Matrix, simplify
from sympy.physics.mechanics import *
from sympy_util import RHSWrapper

###################################################################################################
# Define the dynamics symbols
###################################################################################################
m = symbols('m:3')          # mass of the pendulum
l = symbols('l:3')          # length of the pendulum
g = symbols('g')            # acceleration due to gravity
mu = symbols('mu:3')        # coefficient of friction
T = dynamicsymbols('tau:2') # torques from the motors
            
q = Matrix(dynamicsymbols('theta:' + str(3))) # q is the state vector
dq = q.diff() # and dq is the velocity

# create the list of parameters
params = list(T) + list(m) + list(l) + list(mu) + [g]

# create values for the parameters
motor_forces = np.zeros_like(T)
masses = np.ones_like(m) * 1.
lengths = np.ones_like(l) * 1.
coefficients_of_friction = np.ones_like(mu) * 0.1

# put all of those values into a list
values = np.concatenate([motor_forces,
                         masses,
                         lengths,
                         coefficients_of_friction,
                         [9.8]])

###################################################################################################
# Describe the system
###################################################################################################
N = ReferenceFrame('N') # Inertial Reference Frame
O = Point('O') # Define a world coordinate origin
O.set_vel(N, 0) # set the velocity of the origin to zero in the world frame


forcelist  = []
segments = []
pivot = O # the first link is attached the origin. the rest are chained
point_pairs = []

x = symbols('x') # symbol for use in integration
for i in xrange(3):
    # create a reference frame for the pendulum
    R = N.orientnew('R' + str(i), 'Axis', [q[i], N.z])
    R.set_ang_vel(N, dq[i]*N.z) # set the angular velocity of R to dq

    # define a point for the center of mass of the pendulum
    C = pivot.locatenew('C' + str(i), R.x*l[i]/2.)
    C.v2pt_theory(pivot, N, R) # solve for and set the velocity of C

    # calculate the inertia (modeled as a uniform density stick rotating at the end)
    # integrate the density (m/l) time the radius squared along the length of the segment
    I_component = integrate(x**2 * m[i]/l[i], (x, 0, l[i]))
    # the pendulum is a 3d model, but has no Ixx inertia because it's zero diamter
    I = inertia(R, 0, I_component, I_component)

    # and define a rigid body for the segment
    segment = RigidBody('segment' + str(i), C, R, m[i], (I , pivot))

    # calculate the potential energy of the segment
    h = (C.pos_from(O)).express(N).args[0][0][0]
    segment.potential_energy = m[i]*g*h
    segments.append(segment)
    
    # the angular velocity of each pivot is the difference between it and it's mounting point
    pivot_vel = dq[0] if i > 0 else 0
    # the force of friction acts on the pendulums reference frame and is proportional to dq
    force_of_friction = (R, - N.z * mu[i] * (dq[i] - pivot_vel))
    forcelist.append(force_of_friction)
    
    # create a point for the end of the bar
    E = pivot.locatenew('E' + str(i), R.x*l[i])
    E.v2pt_theory(pivot, N, R) # solve for and set the velocity
    point_pairs.append((pivot, E))
    
    # create a new pivot for the next two segments
    if i == 0:
        pivot = E
    else:
        forcelist.append([R, N.z * T[i-1]])

###################################################################################################
# Derive the dynamics and wrap
###################################################################################################
L = Lagrangian(N, *segments) # create the lagrangian L = T - V
lm = LagrangesMethod(L, [q], forcelist=forcelist, frame=N)
le = lm.form_lagranges_equations()
rhs = lm.rhs() # extract the right hand side

# RHSWrapper handles evaluating the dynamics model 
f = RHSWrapper(rhs, list(q) + list(dq), params, values)

###################################################################################################
# Performance eval
# ###################################################################################################
# from time import time
# x = np.random.rand(6)
# rhs_subbed = msubs(rhs, dict(zip(params, values)))
# rhs_subbed_simplified = Matrix([expr.simplify() for expr in rhs_subbed])


# start = time()
# f.step(np.random.rand(6), 0)
# print "RHSWrapper:", time() - start

# start = time()
# rhs_subbed.evalf(subs=dict(zip(list(q) + list(dq), x)))
# print "evalf:", time() - start

# start = time()
# rhs_subbed_simplified.evalf(subs=dict(zip(list(q) + list(dq), x)))
# print "evalf simplified:", time() - start

# for n in xrange(20):
# 	start = time()
# 	[expr.evalf(subs=dict(zip(list(q) + list(dq), x)), n=n) for expr in rhs_subbed_simplified]
# 	print "evalf:",n, time() - start

# RESULTS:
# RHSWrapper: 0.000677108764648
# evalf: 32.5150971413
# evalf simplified: 11.7823700905
# evalf: 0 10.725591898
# evalf: 1 10.8746469021
# evalf: 2 10.9188129902
# evalf: 3 10.8037390709
# evalf: 4 10.8151569366
# evalf: 5 11.5167958736
# evalf: 6 14.280561924
# evalf: 7 12.6007838249
# evalf: 8 11.6691598892
# evalf: 9 11.3944509029
# evalf: 10 11.2580518723
# evalf: 11 12.1907160282
# evalf: 12 11.1282548904
# evalf: 13 11.2689099312
# evalf: 14 11.3678410053
# evalf: 15 11.4736580849
# evalf: 16 11.2112388611
# evalf: 17 11.3119528294
# evalf: 18 12.3446800709
# evalf: 19 11.4753730297
