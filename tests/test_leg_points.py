import numpy as np
from math import *
import random

l1=50
l2=20
l3=120
l4=155

def calcLegPoints(angles):
    (_l1,_l2,_l3,_l4)=(l1,l2,l3,l4)
    (theta1,theta2,theta3)=angles
    theta23=theta2+theta3

    T0=np.array([0,0,0,1])
    T1=T0+np.array([-_l1*cos(theta1),_l1*sin(theta1),0,0])
    T2=T1+np.array([-_l2*sin(theta1),-_l2*cos(theta1),0,0])
    T3=T2+np.array([-_l3*sin(theta1)*cos(theta2),-_l3*cos(theta1)*cos(theta2),_l3*sin(theta2),0])
    T4=T3+np.array([-_l4*sin(theta1)*cos(theta23),-_l4*cos(theta1)*cos(theta23),_l4*sin(theta23),0])

    return np.array([T0,T1,T2,T3,T4])


a = np.array([90, 45, 30])
b = calcLegPoints(a)
print(b)