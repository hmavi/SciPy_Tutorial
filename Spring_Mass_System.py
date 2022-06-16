import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

##spring mass system # 2 masses attached to 3 springs

#  |<><><><><>[mass1]<><><><><>[mass2]<><><><><>|

##------PROBLEM DEFINITION------------------

m1 = 1    #mass1 (kg)
m2 = 1    #mass2 (kg)
k = 10   #spring constant (kg/sec^2)

#       y1(0)      y1'(0)        y2(0)       y2'(0)
z0 = [   1,     np.sqrt(3*k),    1,     -np.sqrt(3*k)]

Ti = 0
Tf = 4.75 #time (sec)
Tstep = 1000
t = np.linspace(Ti,Tf,Tstep)

##------MAIN FUNCTION-----------------------

def model(z,t):
    
    y1 = z[0]
    u1 = z[1]
    y2 = z[2]
    u2 = z[3]
    
    dy1 = u1
    du1 = (-k*y1 + k*(y2-y1))/m1
    dy2 = u2
    du2 = (-k*(y2-y1) - k*y2)/m2
    
    return [dy1,du1,dy2,du2]

z = odeint(model,z0,t)   #calling the main function

##------POST PROCESSING--------------------
#print(z)

Y1 = z[:,0]
Y2 = z[:,2]

plt.xlim([0,Tf])
plt.ylim([-2.1,2.1])
plt.plot(t,Y1,'b--')
plt.plot(t,Y2,'r-')
plt.grid()
plt.xlabel('time')
plt.legend(['Y1','Y2'])

plt.show()
