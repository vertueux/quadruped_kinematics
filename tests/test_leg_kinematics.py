import numpy as np
from math import *
import random

l1=50
l2=20
l3=120
l4=155

def legIK(point):
        (x,y,z)=(point[0],point[1],point[2])
        (_l1,_l2,_l3,_l4)=(l1,l2,l3,l4)
        try:
            F=sqrt(x**2+y**2-_l1**2)
        except ValueError:
            print("Error in legIK with x {} y {} and _l1 {}".format(x,y,_l1))
            F=_l1
        G=F-_l2
        H=sqrt(G**2+z**2)
        print(H)
        theta1=-atan2(y,x)-atan2(F,-_l1)
        print(theta1)
        D=(H**2-_l3**2-_l4**2)/(2*_l3*_l4)
        print(D)
        try:
            theta3=acos(D)
        except ValueError:
            print("Error in legIK with x {} y {} and D {}".format(x,y,D))
            theta3=0
        theta2=atan2(z,G)-atan2(_l4*sin(theta3),_l3+_l4*cos(theta3))

        return(theta1,theta2,theta3)

a = np.array([1200, 1200, 1200])
b = legIK(a)

print(b)