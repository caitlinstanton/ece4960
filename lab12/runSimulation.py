# RUN ME
# This file gives the initial condition to the scipy.integrat.odeint function
# and plots the resulting state outputs at each time step in an animation and
# on a plot that compares the actual output with the reference input

import numpy as np
import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import control
import pendulumParam as P
from kalmanFilter import kalmanFilter, kalmanFilterEC
from pendulumNonlinearDynamics import Pendulum
from pendulumAnimation import pendulumAn
from plotDataZ import plotData
from signalGenerator import signalGen

#Compute controller
dpoles = np.array([-1.1, -1.2, -1.3, -1.4])
Kr = control.place(P.A * 1.01, P.B * 1.01, dpoles)

print(control.obsv(P.A, P.C))  # which states of A, C are observable?
print(np.linalg.matrix_rank(control.obsv(
    P.A, P.C)))  # expect a rank of 0 if observable

#Initialize and rename for convenience
ref = signalGen(amplitude=.5, frequency=0.05, y_offset=0)
pendulum = Pendulum(param=P)

states = [np.array([[P.z0], [P.zdot0], [P.theta0], [P.thetadot0]])]
states_est = [states[0]]
mu = np.array([[P.z0 + 0.1], [P.zdot0 + 0.1], [P.theta0 + 0.1],
               [P.thetadot0 + 0.1]])
sigma = np.eye(4) * 0.00001
u = 0

#performs very simple first order integration
length = int((P.t_end - P.t_start) /
             P.Ts)  #The number of time steps over the time interval
t_array = np.linspace(P.t_start, P.t_end,
                      length)  #The time vector for integration.
dt = t_array[1] - t_array[0]

deadband = (120 / 255) * (3.33) * (
    0.522)  #~0.8 = deadband of robot/motorval (from lab 6) * acc * mass
saturation = 0.522 * 3.33

for t in t_array[:-1]:
    des_state = np.array([[ref.square(t)[0]], [0.0], [0.0], [0.0]])
    old_state = states[-1]

    #Update controller and sensors every <T_update> seconds
    if (t % P.T_update) < dt:
        u = -Kr.dot(mu - des_state)

        if u > 0:
            if u < deadband:
                u = 0
            elif u > saturation:
                u = saturation
        else:
            if u > (-1 * deadband):
                u = 0
            elif u < (-1 * saturation):
                u = -1 * saturation

        # y_kf = P.C.dot(
        #     old_state) + np.random.randn() * 0.01  # to add sensor noise
        y_kf = P.C.dot(old_state) + np.random.randn(
            2, 1) * 0.01  # to add sensor noise to z, thetadot
        mu, sigma = kalmanFilterEC(mu, sigma, u, y_kf, t)

    new_state = old_state + np.array(pendulum.cartpendfunc(old_state, u)) * dt

    #Arrays for debugging
    states_est.append(mu)
    states.append(new_state)

#animation
plt.close('all')
animation = pendulumAn()

#Recast arrays for plotting
states = np.array(states)
states_est = np.array(states_est)

i = 0
reference = np.zeros((length, 1))
while i < len(t_array):
    #calculate the reference value
    t_curr = t_array[i]  #Current time step
    z_ref = ref.square(t_curr)  #Get cart z location
    reference[i:(i + 100)] = z_ref  #Which reference value is used

    #update animation and data plots
    animation.drawPendulum(states[i, :])  #Animate
    plt.pause(0.001)  #Pause while plot updates
    i = i + 100  #speeds up the simulation by not plotting all the points

# Plot how closely the actual performance of the pendulum on a cart matched the desired performance
dataPlot = plotData()  #initializes plot
dataPlot.Plot(t_array, reference, states_est, 1)  #plot the estimate
dataPlot.Plot(t_array, reference, states, 2)  #plots the data
plt.pause(10)

# dataPlot.PlotUncertainty(t_array, reference, states_est,
#                          1)  #plot the uncertainty
# dataPlot.PlotUncertainty(t_array, reference, states, 2)  #plots the data
# plt.pause(10)