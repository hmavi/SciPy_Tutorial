import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from control.matlab import lqr

##------PROBLEM DEFINITION------------------

#parameter definition
Rm = 8.4    #motor resistance (Ohm)
kt = 0.042  #current torrue (N-m/A)
km = 0.042  #Back-emf constant (V-s/rad)

Mr = 0.095  #rotary arm mass (kg)
Lr = 0.085  #total rotary arm length (m)
Jr = Mr*(Lr**2)/12  #Moment of inertia about pivot (kg-m^2)
Dr = 0      #Equivalent Viscous Damping Coefficient (N-m-s/rad)

Mp = 0.024  #pendulum mass (kg)
Lp = 0.129  #total pendulum length (m)
Jp = Mp*(Lp**2)/12  #Moment of inertia about pivot (kg-m^2)
Dp = 0      #Equivalent Viscous Damping Coefficient (N-m-s/rad)

g  = 9.81   #gravity constant (m/sec^2)
Jt = Jr*Jp  +  Mp*((Lp/2)**2)*Jr  +  Jp*Mp*(Lr**2)  #Total Inertia (kg^2-m^4)

#model definition
    #state variable
        #theta  - rotary arm angle
        #alpha  - pendulum angle
        #dtheta - rotary arm angular velocity
        #dalpha - pendulum angular velocity

#State Space representation
A = np.array([
               [0,  0,                                1,                         0                    ],
               [0,  0,                                0,                         1                    ],
               [0, (Mp**2)*((Lp/2)**2)*Lr*g/Jt,      -Dr*(Jp+Mp*(Lp/2)**2)/Jt,  -Mp*(Lp/2)*Lr*Dp/Jt   ],
               [0,  Mp*g*(Lp/2)*(Jr+Mp*(Lr**2))/Jt,  -Mp*(Lp/2)*Lr*Dr/Jt,       -Dp*(Jr+Mp*(Lr**2))/Jt]
            ])

B = np.array([
                [  0                       ],
                [  0                       ],
                [ (Jp + Mp*((Lp/2)**2))/Jt ],
                [  Mp*(Lp/2)*Lr/Jt         ]
            ])

#Add actuator dynamics
A[0,2] = A[0,2] - km*km/Rm*B[0,0]
A[1,2] = A[1,2] - km*km/Rm*B[1,0]
A[2,2] = A[2,2] - km*km/Rm*B[2,0]
A[3,2] = A[3,2] - km*km/Rm*B[3,0]
#print(A)

B = km*B/Rm
#print(B)

#state weighting matrix
Q = np.array([
                [25, 0, 0, 0 ],
                [0,  1, 0, 0 ],
                [0,  0, 1, 0 ],
                [0,  0, 0, 10]
                ])

#control cost matrix
R = np.array([  [ 1 ]  ])

K,P,E = lqr(A, B, Q, R)  #implementing LQR controller
#print(K)   #control gain
#print(P)   #solution to riccati equation
#print(E)   #Eigenvales for closed loop

##------MAIN FUNCTION-----------------------

z0 = [-20,0,0,0]  #initial valve for state variables

Ti = 0
Tf = 4 #time (sec)
Tstep = 2000
t = np.linspace(Ti,Tf,Tstep)  #setting time span

zd = [20,0,0,0]  #desired valve for state variables

def model(z,t):
    
    theta = z[0]
    alpha = z[1]
    v = z[2]
    w = z[3]
    #control law
    u = K[0,0]*(zd[0]-theta) + K[0,1]*(zd[1]-alpha) + K[0,2]*(zd[2]-v) + K[0,3]*(zd[3]-w)
    
    dtheta = v
    dalpha = w
    dv     = A[2,0]*theta + A[2,1]*alpha + A[2,2]*v + A[2,3]*w + B[2,0]*u
    dw     = A[3,0]*theta + A[3,1]*alpha + A[3,2]*v + A[3,3]*w + B[3,0]*u
    
    return [dtheta,dalpha,dv,dw]

z = odeint(model,z0,t)   #calling the main function

##------POST PROCESSING--------------------

#print(z)
Theta = z[:,0]
Alpha = z[:,1]

#display Theta
fig1 = plt.subplot(2,1,1)
plt.xlim([0,Tf])
plt.ylim([-24,22.5])
plt.plot(t,Theta,'b-.')
plt.grid()
plt.xlabel('time')
plt.ylabel('theta')

#display Alpha
fig2 = plt.subplot(2,1,2)
plt.xlim([0,Tf])
plt.ylim([-3.1,1.6])
plt.plot(t,Alpha,'r--')
plt.grid()
plt.xlabel('time')
plt.ylabel('alpha')

plt.show()
