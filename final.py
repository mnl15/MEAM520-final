#!/usr/bin/python2
from time import sleep
import numpy as np
import rospy
import sys
from random import random as rand
from grabstatic import grabstatic
from calcstaticpath import calcstaticpath
from calculateFK import calculateFK
from IK_velocity import IK_velocity
import math
import sys
from os import getcwd
sys.path.append(getcwd() + "/../Core")

from arm_controller import ArmController

if __name__=='__main__':
    if len(sys.argv) < 2:
        print('usage: python final.py <color>')
        sys.exit()

    color = sys.argv[1]
    lynx = ArmController(color)

    lynx.wait_for_start()
    sleep(1) # wait for setup

    # get state of your robot
    [q, qd]  = lynx.get_state()

    # get state of scoreable objects
    [name, pose, twist] = lynx.get_object_state()

    # swiping
    qswiping = [np.array([1.228, 0.746, -0.161, -0.585, 0., 30.]), np.array([0.89, 1.25, -1.022, -0.228, -1.571, 30.]),
                np.array([0.301, 1.195, -0.926, -0.27, 0., 30.]), np.array([0.398, 0.155, 0.693, -0.848, 0., 15.]),
                np.array([0.89, 1.25, -1.022, -0.228, -1.571, 30.]), np.array([0.366, 0.719, -0.116, -0.602, 0., 30.])]
    for q in qswiping:
        lynx.command(q)
        [q1, qd] = lynx.get_state()
        rospy.sleep(2)

    indexlist=[]

    #find all static blocks depending on team color
    lynx.command(np.array([0, 0, 0, 0, 0, 0]))
    sleep(2)

    #*******************TO MONA THIS IS ALL NEW CODE ADD IN SWIPE AND STARTING GUN ABOVE THIS*********************
    indexlist=[]

    lynx.command(np.array([0, 0, 0, 0, 0, 0]))
    rospy.sleep(2)

    #find all static blocks depending on team color
    for x in range(0, 13):
        if 'static' in name[x]:
            staticpose=pose[x]
            if 'red' in color:
                if staticpose[1, 3]<0:
                    indexlist.append(x)
            else:
                if staticpose[1, 3]>0:
                    indexlist.append(x)

    #find the closest block to the gripper in terms xwise
    closelist=[]
    for x in range(0, len(indexlist)):
        staticpose=pose[indexlist[x]]
        closelist.append(staticpose[0,3])

    #sort the list from closest to furthest
    closelistsorted=list(closelist)
    closelistsorted.sort()

    #loop through each of the static blocks
    for x in range(0, len(closelistsorted)):
        #get the updated posed of the block
        [name, pose, twist] = lynx.get_object_state()

        #Change the block poses into the robot frame
        if 'red' in color:
            blockindex=closelist.index(closelistsorted[x])
            staticpose=pose[indexlist[blockindex]]
            staticpose[0, 3]=staticpose[0, 3]+200
            staticpose[1, 3]=staticpose[1, 3]+200

            #calculate the qpath to grab the block
            qpathtograb, type, redo=grabstatic(staticpose)

            #if we receive a flag that says the block will need to be rotated again after this movement (if the block started out with white side on the top or the bottom or was out of reach of the gripper)
            if redo==1:
                #complete the first rotation movement
                for q in qpathtograb:
                    lynx.command(q)
                    [q1, qd] = lynx.get_state()
                    tolerance = 0.1
                    detect=0
                    pathmatrix=np.array([0, 0, 0, 0, 0, 0])
                    for y in range(0, 10000):
                        if y>=7:
                            if ((abs(pathmatrix[y-6, :] - pathmatrix[y, :]) <= tolerance).all()):
                                break
                        rospy.sleep(.01)
                    	[q1, qd] = lynx.get_state()
                        pathmatrix=np.vstack((pathmatrix, q1))
                rospy.sleep(.5)

                #get the new pose of the block and calculate the qpath to grab the block again
                [name, pose, twist] = lynx.get_object_state()
                staticpose=pose[indexlist[blockindex]]
                staticpose[0, 3]=staticpose[0, 3]+200
                staticpose[1, 3]=staticpose[1, 3]+200
                qpathtograb, type, redo=grabstatic(staticpose)

                #if we are still not done with rotating the block
                if redo==1:
                    #complete the second rotation movement
                    for q in qpathtograb:
                        lynx.command(q)
                        [q1, qd] = lynx.get_state()
                        tolerance = 0.1
                        detect=0
                        pathmatrix=np.array([0, 0, 0, 0, 0, 0])
                        for y in range(0, 10000):
                            if y>=7:
                                if ((abs(pathmatrix[y-6, :] - pathmatrix[y, :]) <= tolerance).all()):
                                    break
                            rospy.sleep(.01)
                            [q1, qd] = lynx.get_state()
                            pathmatrix=np.vstack((pathmatrix, q1))
                    rospy.sleep(.5)

                    #get the new pose of the block and calculate the qpath to grab the block again
                    [name, pose, twist] = lynx.get_object_state()
                    staticpose=pose[indexlist[blockindex]]
                    staticpose[0, 3]=staticpose[0, 3]+200
                    staticpose[1, 3]=staticpose[1, 3]+200
                    qpathtograb, type, redo=grabstatic(staticpose)

        #same as above except for team blue
        else:
            blockindex=closelist.index(closelistsorted[3-x])
            staticpose=pose[indexlist[blockindex]]
            staticpose[0, 3]=200-staticpose[0, 3]
            staticpose[1, 3]=200-staticpose[1, 3]
            staticpose[0,0:3]=-staticpose[0,0:3]
            staticpose[1,0:3]=-staticpose[1,0:3]
            qpathtograb, type, redo=grabstatic(staticpose)
            if redo==1:
                for q in qpathtograb:
                    lynx.command(q)
                    [q1, qd] = lynx.get_state()
                    tolerance = 0.1
                    detect=0
                    pathmatrix=np.array([0, 0, 0, 0, 0, 0])
                    for y in range(0, 10000):
                        if y>=7:
                            if ((abs(pathmatrix[y-6, :] - pathmatrix[y, :]) <= tolerance).all()):
                                break
                        rospy.sleep(.01)
                    	[q1, qd] = lynx.get_state()
                        pathmatrix=np.vstack((pathmatrix, q1))
                rospy.sleep(.5)
                [name, pose, twist] = lynx.get_object_state()
                staticpose=pose[indexlist[blockindex]]
                staticpose[0, 3]=200-staticpose[0, 3]
                staticpose[1, 3]=200-staticpose[1, 3]
                staticpose[0,0:3]=-staticpose[0,0:3]
                staticpose[1,0:3]=-staticpose[1,0:3]
                qpathtograb, type, redo=grabstatic(staticpose)
                if redo==1:
                    for q in qpathtograb:
                        lynx.command(q)
                        [q1, qd] = lynx.get_state()
                        tolerance = 0.1
                        detect=0
                        pathmatrix=np.array([0, 0, 0, 0, 0, 0])
                        for y in range(0, 10000):
                            if y>=7:
                                if ((abs(pathmatrix[y-6, :] - pathmatrix[y, :]) <= tolerance).all()):
                                    break
                            rospy.sleep(.01)
                            [q1, qd] = lynx.get_state()
                            pathmatrix=np.vstack((pathmatrix, q1))
                    rospy.sleep(.5)
                    [name, pose, twist] = lynx.get_object_state()
                    staticpose=pose[indexlist[blockindex]]
                    staticpose[0, 3]=200-staticpose[0, 3]
                    staticpose[1, 3]=200-staticpose[1, 3]
                    staticpose[0,0:3]=-staticpose[0,0:3]
                    staticpose[1,0:3]=-staticpose[1,0:3]
                    qpathtograb, type, redo=grabstatic(staticpose)
        if type==7:
            continue
        number=0

        #iterate through the qpath
        for q in qpathtograb:
            lynx.command(q)
            [q1, qd] = lynx.get_state()
            tolerance = 0.1
            detect=0
            pathmatrix=np.array([0, 0, 0, 0, 0, 0])

            #if the robot is in the same position for 6 straight iterations then we are at final position and can move onto the next
            for y in range(0, 10000):
                if y>=7:
                    if ((abs(pathmatrix[y-6, :] - pathmatrix[y, :]) <= tolerance).all()):
                        break
                rospy.sleep(.01)
            	[q1, qd] = lynx.get_state()
                pathmatrix=np.vstack((pathmatrix, q1))

        #calculate the path from the block to the stack
        qpathtoplace=calcstaticpath(qpathtograb[-1], x, type)
        number=0

        #iterate through the qpath
        for q in qpathtoplace:
            lynx.command(q)
            [q1, qd] = lynx.get_state()
            tolerance = 0.1
            detect=0
            pathmatrix=np.array([0, 0, 0, 0, 0, 0])
            for x in range(0, 10000):
                if x>=7:
                    if ((abs(pathmatrix[x-6, :] - pathmatrix[x, :]) <= tolerance).all()):
                        break
                rospy.sleep(.01)
            	[q1, qd] = lynx.get_state()
                pathmatrix=np.vstack((pathmatrix, q1))


    #playing defense and taunting the opponent after all of our blocks are stacked
    first, _=lynx.get_state()
    first[2]=first[2]-math.pi/6
    qtaunting=first
    qtaunt1=np.array([0, math.pi/3, -math.pi/4, 0, 0, 30.])
    qtaunt2=np.array([1.35, math.pi/3, -math.pi/4, 0, 0, 30.])
    for i in range(0, 100):
        qtaunting=np.vstack((qtaunting, qtaunt1))
        qtaunting=np.vstack((qtaunting, qtaunt2))

    for q in qtaunting:
        lynx.command(q)
        [q1, qd] = lynx.get_state()
        tolerance = 0.1
        detect=0
        pathmatrix=np.array([0, 0, 0, 0, 0, 0])
        for x in range(0, 10000):
            if x>=15:
                if ((abs(pathmatrix[x-14, :] - pathmatrix[x, :]) <= tolerance).all()):
                    break
            rospy.sleep(.01)
            [q1, qd] = lynx.get_state()
            pathmatrix=np.vstack((pathmatrix, q1))
    lynx.stop()
