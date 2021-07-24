import numpy as np
import math

def calcJacobian(q, joint):
    lengths=np.array([76.2, 146.05, 187.325, 34.0, 34.0])
    if joint==0:
        jointi=joint
    else:
        jointi=joint-1

    parameters=np.array([[0.0, -(math.pi)/2.0, lengths[0], q[0]],
                        [lengths[1], 0.0, 0.0, (q[1]-(math.pi/2.0))],
                        [lengths[2], 0.0, 0.0, (q[2]+(math.pi/2.0))],
                        [0.0, math.pi/2.0, 0.0, (q[3]+(math.pi/2.0))],
                        [0.0, 0.0, lengths[3]+lengths[4], (q[4]+math.pi)]])

    transformlist=[]

    for x in range(0, 5):
        theta=parameters[x, 3]
        alpha=parameters[x, 1]
        a=parameters[x, 0]
        d=parameters[x, 2]

        transformlist.append(np.array([[math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
                                    [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
                                    [0.0, math.sin(alpha), math.cos(alpha), d],
                                    [0.0, 0.0, 0.0, 1.0]]))

    T01=transformlist[0]
    T02=transformlist[0].dot(transformlist[1])
    T03=transformlist[0].dot(transformlist[1]).dot(transformlist[2])
    T04=transformlist[0].dot(transformlist[1]).dot(transformlist[2]).dot(transformlist[3])
    T05=transformlist[0].dot(transformlist[1]).dot(transformlist[2]).dot(transformlist[3]).dot(transformlist[4])
    transmat=[np.identity(4), T01, T02, T03, T04, T05]

    jacobianv=np.zeros((3, jointi))
    jacobianw=np.zeros((3, jointi))

    if joint==5:
        on=transmat[jointi].dot([0.0, 0.0, 34.0, 1.0])[0:3]
    else:
        on=transmat[jointi].dot([0.0, 0.0, 0.0, 1.0])[0:3]

    for x in range(0, jointi):
        oi1=transmat[x].dot([0.0, 0.0, 0.0, 1.0])[0:3]
        diff=on-oi1
        zi1=transmat[x][:, 2]
        zi1=zi1[0:3]
        jacobianw[:, x]=zi1
        jacobianv[:, x]=np.cross(zi1, diff)

    if joint==1 or joint==0:
        jacobianv=np.zeros((3, 1))
        jacobianw=np.zeros((3,1))
        
    return jacobianv, jacobianw
