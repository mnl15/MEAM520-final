import numpy as np
import math
from calcJacobian import calcJacobian
from FK_velocity import FK_velocity
from IK_velocity import IK_velocity
from calculateFK import calculateFK

def calcstaticpath(qstart, num, type):
    calcFK=calculateFK()
    blockstart, T0e=calcFK.forward(qstart)
    xfinal=100.
    yfinal=-300.
    gripheight=11.

    if type==10:
        xfinal=100.
        yfinal=-300+5.
        zfinal=gripheight
    else:
        zfinal=0

    #calculate the position of the end effector
    xstart=blockstart[5, 0]
    ystart=blockstart[5, 1]
    zstart=blockstart[5, 2]
    blockheight=20.
    zclearance=150.

    #if the gripper is gripping the block from the side then adjust the final goal positions
    if type==1:
        xfinal=xfinal-3
        yfinal=yfinal+8

    #arrays for first movement where we bring the gripper to a z clearance value while keeping it parallel to the platform surfaces
    v1=np.array([0, 0, zclearance-zstart])
    omega1=np.array([0, 0, float("NaN")])

    #arrays for the second movement where we bring the gripper to a z clearance value above the final goal while keeping the gripper parallel to the platform surface
    v2=np.array([xfinal-xstart, yfinal-ystart, 0])
    omega2=np.array([0, 0, float("NaN")])

    #arrays for the third movement where we bring the gripper to its final position based on how many blocks have already been stacked
    v3=np.array([0, 0, (40+zfinal+((num)*blockheight))-zclearance])
    omega3=np.array([0, 0, float("NaN")])

    #go to clearance
    q=np.copy(qstart)
    q[5]=-15.
    for y in range(0, 250):
        jointpositions, _ =calcFK.forward(q)
        dq=IK_velocity(q, v1/250., omega1, 6)
        for x in range (0, 6):
            q[x]=q[x]+dq[x]
    qnext=q+0
    qpath=qnext

    #go to clearance above goal
    for y in range(0, 250):
        jointpositions, _ =calcFK.forward(q)
        dq=IK_velocity(q, v2/250., omega2, 6)
        for x in range (0, 6):
            q[x]=q[x]+dq[x]
    qnext=q+0
    qpath=np.vstack((qpath, qnext))

    #go to goal
    for y in range(0, 250):
        jointpositions, _ =calcFK.forward(q)
        dq=IK_velocity(q, v3/250., omega3, 6)
        for x in range (0, 6):
            q[x]=q[x]+dq[x]
    qnext=q+0
    qpath=np.vstack((qpath, qnext))

    #open gripper
    qnext[5]=30.
    qpath=np.vstack((qpath, qnext))

    #move back up to clearance position
    qnext=np.copy(qpath[1])
    qnext[5]=30
    qpath=np.vstack((qpath, qnext))

    return qpath
