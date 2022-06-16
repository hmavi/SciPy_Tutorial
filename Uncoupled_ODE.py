import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

##------PROBLEM DEFINITION------------------
z0 = [2.78,-0.43]  ##initial condition y(0),y'(0)
ti = 0        ## start time
tf = 20       ## end time
tstep = 200   ## step
t = np.linspace(ti,tf,tstep)   ##setting the interval for integration

##------MAIN FUNCTION-----------------------
def model(z,t):
    y = z[0]
    p = z[1]
    
    dy = p
    dp = 2*np.cos(t) - 0.25*np.sin(t) + 0.09*t - 0.75*y - 2*p
    return [dy,dp]

z = odeint(model,z0,t)   #calling the main function

##------POST PROCESSING--------------------
Y = z[:,0]
dY = z[:,1]

plt.plot(t,Y,'b--')
plt.xlabel('t')
plt.ylabel('y')
plt.grid()
plt.show()
