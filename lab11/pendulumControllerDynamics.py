# This file contains the scipy.integrate function called in the runSimulation.py file.
# I also added the force contraint function here and in the kalman filter (when it's being called)

import numpy as np
from signalGenerator import signalGen
import control
import pendulumParam as p
import scipy
import random


class pendulumCnt:
    def __init__(self,
                 m1=None,
                 m2=None,
                 ell=None,
                 b=None,
                 g=None,
                 Ts=None,
                 angle_limit=None,
                 K=None,
                 param=None,
                 zref=None):
        #Initial state conditions

        # Mass of the pendulum, kg
        if not m1 is None:
            self.m1 = m1
        elif not param is None:
            self.m1 = param.m1

        # Mass of the cart, kg
        if not m2 is None:
            self.m2 = m2
        elif not param is None:
            self.m2 = param.m2

        # Length of the rod, m
        if not ell is None:
            self.ell = ell
        elif not param is None:
            self.ell = param.ell

        # Damping coefficient, Ns
        if not b is None:
            self.b = b
        elif not param is None:
            self.b = param.b

        # Gravity constant
        if not g is None:
            self.g = g
        elif not param is None:
            self.g = param.g

        # sample rate of controller
        if not Ts is None:
            self.Ts = Ts
        elif not param is None:
            self.Ts = param.Ts

        if not K is None:
            self.K = K
        elif not param is None:
            if 'K' in param.__dict__.keys():
                self.K = param.K
            else:
                self.K = None
        else:
            self.K = None

        self.angle_limit = np.pi * 2.0

        if not zref is None:
            self.zref = zref
        else:
            self.reference = signalGen(amplitude=.5,
                                       frequency=0.05,
                                       y_offset=0)
            # self.zref = self.reference.square
            self.zref = self.reference.sin

    ####################################################
    #               scipy.integrate
    ####################################################
    def cartpendfunc(self, y, t):
        #unpack the state
        z, zdot, theta, thetadot = y

        #don't allow theta to exceed 2pi
        #need this so that equivalnent angles don't cause a difference in
        #in the reference input
        theta = self.limitTheta(theta)

        #Get reference inputs from signal generators
        zref = self.zref(t)  #control cart z location
        # #change to sinusoidal from Campuswire post

        #calculting the new control force
        curr_state = np.array([[z], [zdot], [theta],
                               [thetadot]])  #current state
        des_state = np.array([zref, [0.0], [np.pi], [0.0]])  #desired state

        #Feedback control. If there's no gain it assigns the control to be 0
        '''
        Compute the input self.u
        Your Code Here
        '''

        #Feedback control with sensor noise
        mu = 0
        sig_z = 0.0001
        sig_z_dot = 0.000
        sig_theta = 0.000
        sig_theta_dot = 0.000
        noise = np.array([[random.gauss(mu, sig_z)],
                          [random.gauss(mu, sig_z_dot)],
                          [random.gauss(mu, sig_theta)],
                          [random.gauss(mu, sig_theta_dot)]])
        #print(noise)
        noise_state = np.add(curr_state, noise)
        noise_state[2] = self.limitTheta(noise_state[2])
        #print(noise_state)

        dpoles = np.array([-0.1, -0.2, -0.3,
                           -0.4])  #original dpoles, robot flew off the screen

        # dpoles = np.array([-1, -1.2, -1.3, -1.4
        #                   ])  #secondary dpoles, allowed robot to balance,
        #more negative equals more aggressive controller
        # Kr = control.place(
        #     p.A, p.B,
        #     dpoles)  #use when not using Q,R

        Q = np.matrix([[1, 0.0, 0.0, 0.0], [0.0, 1, 0.0, 0.0],
                       [0.0, 0.0, 10, 0.0], [0.0, 0.0, 0.0, 100]])
        R = np.matrix([1000])

        # DEADBAND AND SATURATION FOR A LIL OF 0.0001 SIGMA NOISE
        Q = np.matrix([[3.0, 0.0, 0.0, 0.0], [0.0, 0.5, 0.0, 0.0],
                       [0.0, 0.0, 0.5, 0.0], [0.0, 0.0, 0.0, 3.0]])
        R = np.matrix([13000.0
                       ])  #want to make this higher to be more aggressive
        #solve algebraic Ricatti equation (ARE)
        S = scipy.linalg.solve_continuous_are(p.A, p.B, Q, R)
        # Solve the ricatti equation and compute the LQR gain
        Kr = np.linalg.inv(R).dot(p.B.transpose().dot(S))  #use when using Q,R

        # f = open("K_poles_lqr_log.txt", "a+")
        # print(Kr, np.linalg.eig(p.A - p.B * Kr)[0])
        # f.write("Poles: " + str(np.linalg.eig(p.A - p.B * Kr)[0]) + ", K: " +
        #         str(Kr) + "\n")
        # f.close()

        # eigenval = np.linalg.eig(p.A - (p.B * Kr))
        #print(eigenval)
        orig_u = Kr * (np.subtract(des_state, curr_state))  #noise_state))
        self.u = orig_u
        #print(self.u) #goes from -1.6 to around 0

        #Feedback control of a non-ideal pendulum on a cart system
        deadband = (120 / 255) * (3.33) * (
            0.522
        )  #~0.8 = deadband of robot/motorval (from lab 6) * acc * mass
        saturation = 0.522 * 3.33

        # print(orig_u)
        if orig_u > 0:
            if orig_u < deadband:
                self.u = np.array([0])
            # elif orig_u > (0.5 * deadband) and orig_u < deadband:
            # self.u = np.array([deadband])
            elif orig_u > saturation:
                self.u = np.array([saturation])
            else:
                self.u = orig_u
        else:
            if orig_u > (-1 * deadband):
                self.u = np.array([0])
            # elif orig_u < (-0.5 * deadband) and orig_u > (-1 * deadband):
            # self.u = np.array([-1 * deadband])
            elif orig_u < (-1 * saturation):
                self.u = np.array([-1 * saturation])
            else:
                self.u = orig_u

        #simplifications for the calculations - constants
        Sy = np.sin(theta)
        Cy = np.cos(theta)
        D = self.m1 * self.ell * self.ell * (self.m2 + self.m1 *
                                             (1.0 - Cy * Cy))

        #calculating state values at the current time step
        ydot0 = zdot
        ydot1 = (1.0 / D) * (
            -self.m1 * self.m1 * self.ell * self.ell * self.g * Cy * Sy +
            self.m1 * self.ell * self.ell *
            (self.m1 * self.ell * thetadot * thetadot * Sy - self.b * zdot)
        ) + self.m1 * self.ell * self.ell * (1.0 / D) * self.u
        ydot2 = thetadot
        ydot3 = (1.0 / D) * (
            (self.m1 + self.m2) * self.m1 * self.g * self.ell * Sy -
            self.m1 * self.ell * Cy *
            (self.m1 * self.ell * thetadot * thetadot * Sy - self.b * zdot)
        ) - self.m1 * self.ell * Cy * (1.0 / D) * self.u
        dydt = [ydot0, ydot1, ydot2, ydot3]
        return dydt

    ####################################################
    #              Extra Functions
    ####################################################
    def limitTheta(self, angle):
        if abs(angle) > self.angle_limit:
            rem = (abs(angle) % self.angle_limit) * np.sign(angle)
        elif abs(angle) <= self.angle_limit:
            rem = angle
        return rem
