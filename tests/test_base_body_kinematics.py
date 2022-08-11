import numpy as np
from math import *
import random

omega = 40
phi = 40
psi = 40
xm = 40
ym = 40
zm = 40

Rx = np.array([[1,0,0,0],[0,np.cos(omega),-np.sin(omega),0],[0,np.sin(omega),np.cos(omega),0],[0,0,0,1]])
Ry = np.array([[np.cos(phi),0,np.sin(phi),0],[0,1,0,0],[-np.sin(phi),0,np.cos(phi),0],[0,0,0,1]])
Rz = np.array([[np.cos(psi),-np.sin(psi),0,0],[np.sin(psi),np.cos(psi),0,0],[0,0,1,0],[0,0,0,1]])
Rxyz = Rx.dot(Ry.dot(Rz))

T = np.array([[0,0,0,xm],[0,0,0,ym],[0,0,0,zm],[0,0,0,0]])
Tm = T+Rxyz