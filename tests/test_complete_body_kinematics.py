import numpy as np
from math import *
import random

L = 140
W = 75

def bodyIK(omega,phi,psi,xm,ym,zm):
        Rx = np.array([[1,0,0,0],[0,np.cos(omega),-np.sin(omega),0],[0,np.sin(omega),np.cos(omega),0],[0,0,0,1]])
        Ry = np.array([[np.cos(phi),0,np.sin(phi),0],[0,1,0,0],[-np.sin(phi),0,np.cos(phi),0],[0,0,0,1]])
        Rz = np.array([[np.cos(psi),-np.sin(psi),0,0],[np.sin(psi),np.cos(psi),0,0],[0,0,1,0],[0,0,0,1]])
        Rxyz = Rx.dot(Ry.dot(Rz))

        T = np.array([[0,0,0,xm],[0,0,0,ym],[0,0,0,zm],[0,0,0,0]])
        Tm = T+Rxyz

        sHp=np.sin(pi/2)
        cHp=np.cos(pi/2)
        (_L,_W)=(L,W)

        return([Tm.dot(np.array([[cHp,0,sHp,_L/2],[0,1,0,0],[-sHp,0,cHp,_W/2],[0,0,0,1]])),
                Tm.dot(np.array([[cHp,0,sHp,_L/2],[0,1,0,0],[-sHp,0,cHp,-_W/2],[0,0,0,1]])),
                Tm.dot(np.array([[cHp,0,sHp,-_L/2],[0,1,0,0],[-sHp,0,cHp,_W/2],[0,0,0,1]])),
                Tm.dot(np.array([[cHp,0,sHp,-_L/2],[0,1,0,0],[-sHp,0,cHp,-_W/2],[0,0,0,1]]))])

a = bodyIK(40.5, 40.5, 40.5, 40.5, 40.5, 40.5)
print(a)