import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib import cm

matplotlib.rcParams['figure.figsize'] = [12, 7]
matplotlib.rcParams['text.usetex'] = False

h = 0.005

# Numerical integration using Runge-Kutta

def integrate( f, T, x0 ):
  X = np.zeros((len(x0), T))
  time = np.zeros(T)
  X[:,0] = x0

  for i in range(T-1):
      k1 = f(time[i], X[:,i])
      k2 = f(time[i] + h/2.0, X[:,i] + h*k1/2.0)
      k3 = f(time[i] + h/2.0, X[:,i] + h*k2/2.0)
      k4 = f(time[i] + h, X[:,i] + h*k3)
      X[:, i+1] = X[:,i] + h*(k1 + 2*k2 + 2*k3 + k4)/6.0
      time[i+1] = time[i] + h
      
  return time, X

# Parameters , see attached document
epsilonAv = 0.0 # range [-0.1, \infty]
epsilonAm = 0.9 # range [-0.1, 0.9]
a  = 1.0
c = 1.0
b = 0.5 # This is the time scale, adjust so the robot behaves for long enough
dp = 0.0
de = 0.0

# Action selection vaariables, do each action when the variable is equal to 1
A_approach = lambda x: np.heaviside(x, 0)
A_explore = lambda x: np.heaviside(-x, 0)

# Model
def f(t, r):
  x1 = r[0]
  y1 = r[1]
  x2 = r[2]
  y2 = r[3]
  ym = (y1 + y2)/2.0
  xm = (x1 + x2)/2.0
  dx1 = -a*(4.0*c*x1**3 - 2.0*x1) - y1 
  dy1 = b*x1- epsilonAm*(y2 + y1 + dp)  - epsilonAv*(y2 - y1 - de) 
  dx2 = -a*(4.0*c*x2**3 - 2.0*x2) - y2
  dy2 = b*x2  - epsilonAm*(y2 + y1 - dp)  - epsilonAv*(y2 - y1 - de)
  return np.array([dx1, dy1, dx2, dy2])

# Integration, run each step per step of simulation in the robot
T = 10000
t, X = integrate( f, T, [-1.0, 1.0, 0.0, 0.0] )

# PLots
fig, ax = plt.subplots(3,1)
ax[0].plot(t, X[0,:])
ax[0].plot(t, X[2,:])
ax[0].set_title('Need')

ax[1].plot(t, X[1,:])
ax[1].plot(t, X[3,:])
ax[1].set_title('Accumulated need')

ax[2].plot(t, A_approach(X[0, :]), 'k')
ax[2].plot(t, A_explore(-X[0, :]), 'r')
ax[2].set_title('Actions')
plt.show()