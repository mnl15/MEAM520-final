import numpy as np
import math
from calcJacobian import calcJacobian
from FK_velocity import FK_velocity
from IK_velocity import IK_velocity
from calculateFK import calculateFK

def calcstaticpath(occ, qstart, plat, num):
    calcFK=calculateFK()
    blockstart, T0e=calcFK.forward(qstart)
    xfinal=100
    yfinal=500
    xstart=blockstart[5, 0]
    ystart=blockstart[5, 1]
    zstart=blockstart[5, 2]
    blockheight=25
    zclearance=100.0

    v1=np.array([0, 0, zclearance])
    omega1=np.array([0, 0, float("NaN")])

    v2=np.array([xfinal-xstart, yfinal-ystart, 0])
    omega2=np.array([0, 0, float("NaN")])

    v3=np.array([0, 0, (num*blockheight)-zclearance])
    omega3=np.array([0, 0, float("NaN")])

    qpath=qstart
    qnext=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    dq=IK_velocity(qstart, v1, omega1, 6)
    for x in range (0, 6):
        qnext[x]=qstart[x]+dq[x]
    qpath=np.vstack((qpath, qnext))

    dq=IK_velocity(qnext, v2, omega2, 6)
    for x in range (0, 6):
        qnext[x]=qnext[x]+dq[x]
    qpath=np.vstack((qpath, qnext))

    dq=IK_velocity(qnext, v3, omega3, 6)
    for x in range (0, 6):
        qnext[x]=qnext[x]+dq[x]
    qpath=np.vstack((qpath, qnext))

    return qpath
