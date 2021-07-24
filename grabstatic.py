import numpy as np
import math
from calcJacobian import calcJacobian
from FK_velocity import FK_velocity
from IK_velocity import IK_velocity
from calculateFK import calculateFK
from calculateIK import calculateIK

def grabstatic(pose_block):
    zclearance=80
    gripheight=11
    redo=0
    calcIK=calculateIK()
    calcFK=calculateFK()

    #get block orientation from axes
    blockx=np.array([pose_block[0, 0], pose_block[1, 0], pose_block[2, 0]])
    blocky=np.array([pose_block[0, 1], pose_block[1, 1], pose_block[2, 1]])
    blockz=np.array([pose_block[0, 2], pose_block[1, 2], pose_block[2, 2]])

    #generate transformation matrix for a downward pose directly above the block
    blockcenter=[pose_block[0, -1], pose_block[1, -1], pose_block[2, -1]]
    down_pose = np.array([[0., 1., 0., 0.], [1., 0., 0., 0.], [0., 0., -1., 0.], [0., 0., 0., 1.]])
    down_pose[0, -1] = blockcenter[0]
    down_pose[1, -1] = blockcenter[1]
    down_pose[2, -1] = blockcenter[2] + zclearance

    #move gripper to downpose over the center of the block
    qnext, errorcheck, _ =calcIK.inverse(down_pose)

    #if this downward orientation is infeasible
    if errorcheck==7:
        #rotate robot so its gripper is parallel with the platform surface
        q=np.array([0, 0, 0, 0, -math.pi/2, 30.])
        qpath=np.copy(q)
        start, T0e=calcFK.forward(q)

        #move gripper to a clearance position above the block
        for y in range(0, 500):
            jointpositions, _ =calcFK.forward(q)
            v=np.array([blockcenter[0]-11-start[5, 0], blockcenter[1]-start[5, 1], 80+blockcenter[2]-start[5, 2]])
            dq=IK_velocity(q, v/500., np.array([0, 0, float("NaN")]), 6)
            for x in range (0, 6):
                q[x]=q[x]+dq[x]
        qnext=q+0
        qpath=np.vstack((qpath, qnext))

        #move gripper to a position that is able to grip the block from the side
        for y in range(0, 500):
            jointpositions, _ =calcFK.forward(q)
            v=np.array([0, 0, -80])
            dq=IK_velocity(q, v/500., np.array([0, 0, float("NaN")]), 6)
            for x in range (0, 6):
                q[x]=q[x]+dq[x]
        qnext=q+0
        qpath=np.vstack((qpath, qnext))

        #close gripper
        qnext[5]=-15
        qpath=np.vstack((qpath, qnext))

        #move the block 40mm closer to the robot base so we can attempt downward orientation again
        q=qnext
        for y in range(0, 500):
            jointpositions, _ =calcFK.forward(q)
            v=np.array([-40, 0, 0])
            dq=IK_velocity(q, v/500., np.array([0, 0, float("NaN")]), 6)
            for x in range (0, 6):
                q[x]=q[x]+dq[x]
        qnext=q+0
        qpath=np.vstack((qpath, qnext))

        #open gripper
        qnext2=np.copy(qnext)
        qnext2[5]=30.
        qpath=np.vstack((qpath, qnext2))

        #move gripper to clearance position and return the path and a flag that signals that we're not done with this block yet
        qnext2[1]=qnext2[1]-math.pi/6
        qpath=np.vstack((qpath, qnext2))
        redo=1
        return qpath, 1, redo

    #if white is on the top or the bottom
    if round(blockz[2], 3)==1.000 or round(blockz[2], 3)==-1.000:
        #rotate gripper so it is parallel with the sides of the block
        qnext=np.append(qnext, [30.])
        qpath=qnext
        newx=np.cross(blockx, [0, 0, -1])
        down_pose[0:3, 0]=newx
        down_pose[0:3, 1]=blockx
        down_pose[2, -1]=blockcenter[2]+gripheight
        qnext, _ , flip=calcIK.inverse(down_pose)
        qnext=np.append(qnext, [30.])
        qpath=np.vstack((qpath, qnext))

        #grip
        qnext[5]=-15.
        qpath=np.vstack((qpath, qnext))

        #if this grip is out of range of the gripper then rotate the block 180 deg so the gripper can reach and manipulate the block
        if flip==1:
            if qnext[4]+math.pi/2<=1.5:
                blockx=-blockx
                qnext[4]=qnext[4]+math.pi/2
                qpath=np.vstack((qpath, qnext))
                qnext[5]=30.
                qnext[4]=qnext[4]-math.pi/2
                qpath=np.vstack((qpath, qnext))
                qnext[5]=-15.
                qpath=np.vstack((qpath, qnext))
                qnext[4]=qnext[4]+math.pi/2
                qpath=np.vstack((qpath, qnext))
            else:
                blockx=-blockx
                qnext[4]=qnext[4]-math.pi/2
                qpath=np.vstack((qpath, qnext))
                qnext[5]=30.
                qnext[4]=qnext[4]+math.pi/2
                qpath=np.vstack((qpath, qnext))
                qnext[5]=-15.
                qpath=np.vstack((qpath, qnext))
                qnext[4]=qnext[4]-math.pi/2
                qpath=np.vstack((qpath, qnext))

            newx=np.cross(blockx, [0, 0, -1])
            down_pose[0:3, 0]=newx
            down_pose[0:3, 1]=blockx
            down_pose[2, -1]=blockcenter[2]+gripheight
            qnext, _ , _=calcIK.inverse(down_pose)
            qnext=np.append(qnext, [30.])
            qpath=np.vstack((qpath, qnext))

            #grip
            qnext[5]=-15.
            qpath=np.vstack((qpath, qnext))


        #rotate so ye is in line with x1
        endloc=blockcenter
        endloc[2]=endloc[2]+gripheight
        normalunit=np.array([0, 0, 1])
        newy=endloc-((np.matmul(endloc, normalunit))*normalunit)
        newymag=math.sqrt(newy[0]**2+newy[1]**2+newy[2]**2)
        newy=newy/newymag
        down_pose[0:3, 1]=newy
        newx=np.cross(newy, [0, 0, -1])
        down_pose[0:3, 0]=newx
        qnext, _ , _ =calcIK.inverse(down_pose)
        qnext=np.append(qnext, [-15.])
        qpath=np.vstack((qpath, qnext))

        #rotate white to side
        newz=newy+0
        newy=np.cross(newz, newx)
        down_pose[0:3, 1]=newy
        down_pose[0:3, 2]=newz
        down_pose[2, -1]=down_pose[2, -1]+50
        qnext, _ , _=calcIK.inverse(down_pose)
        qnext=np.append(qnext, [-15.])
        qpath=np.vstack((qpath, qnext))

        #drop the block so that the white is now on the side
        qnext2=np.copy(qnext)
        qnext2[5]=30.
        qpath=np.vstack((qpath, qnext2))
        qnext2[1]=qnext2[1]-math.pi/6
        qpath=np.vstack((qpath, qnext2))

        #flag that says we're not done with this block yet
        redo=1
        type=1

    #if white is on the side
    else:
        #go to grip in correct orientation (y end effector in line with white side)
        qnext=np.append(qnext, [30.])
        qpath=qnext
        newx=np.cross(blockz, [0, 0, -1])
        down_pose[0:3, 0]=newx
        down_pose[0:3, 1]=blockz
        down_pose[2, -1]=blockcenter[2]+gripheight
        qnext, _ , flip=calcIK.inverse(down_pose)
        qnext=np.append(qnext, [30.])
        qpath=np.vstack((qpath, qnext))
        #grip
        qnext[5]=-15.
        qpath=np.vstack((qpath, qnext))

        #if this orientation is out of range of the gripper then rotate it 180 deg
        if flip==1:
            if qnext[4]+math.pi/2<=1.5:
                blockz=-blockz
                qnext[4]=qnext[4]+math.pi/2
                qpath=np.vstack((qpath, qnext))
                qnext[5]=30.
                qnext[4]=qnext[4]-math.pi/2
                qpath=np.vstack((qpath, qnext))
                qnext[5]=-15.
                qpath=np.vstack((qpath, qnext))
                qnext[4]=qnext[4]+math.pi/2
                qpath=np.vstack((qpath, qnext))
            else:
                blockz=-blockz
                qnext[4]=qnext[4]-math.pi/2
                qpath=np.vstack((qpath, qnext))
                qnext[5]=30.
                qnext[4]=qnext[4]+math.pi/2
                qpath=np.vstack((qpath, qnext))
                qnext[5]=-15.
                qpath=np.vstack((qpath, qnext))
                qnext[4]=qnext[4]-math.pi/2
                qpath=np.vstack((qpath, qnext))

            newx=np.cross(blockz, [0, 0, -1])
            down_pose[0:3, 0]=newx
            down_pose[0:3, 1]=blockz
            down_pose[2, -1]=blockcenter[2]+gripheight
            qnext, _ , _=calcIK.inverse(down_pose)
            qnext=np.append(qnext, [30.])
            qpath=np.vstack((qpath, qnext))
            #grip
            qnext[5]=-15.
            qpath=np.vstack((qpath, qnext))

        #rotate so ye is in line with x1
        endloc=blockcenter
        endloc[2]=endloc[2]+gripheight
        normalunit=np.array([0, 0, 1])
        newy=endloc-((np.matmul(endloc, normalunit))*normalunit)
        newymag=math.sqrt(newy[0]**2+newy[1]**2+newy[2]**2)
        newy=newy/newymag
        down_pose[0:3, 1]=newy
        newx=np.cross(newy, [0, 0, -1])
        down_pose[0:3, 0]=newx
        qnext, _ , _ =calcIK.inverse(down_pose)
        qnext=np.append(qnext, [-15.])
        qpath=np.vstack((qpath, qnext))

        #rotate white to side
        newz=newy+0
        newy=np.cross(newz, newx)
        down_pose[0:3, 1]=newy
        down_pose[0:3, 2]=newz
        down_pose[2, -1]=down_pose[2, -1]+50
        qnext, _ , _=calcIK.inverse(down_pose)
        qnext=np.append(qnext, [-15.])
        qpath=np.vstack((qpath, qnext))
        type=2

    return qpath, type, redo
