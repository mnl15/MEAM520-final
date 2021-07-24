import numpy as np
from calcJacobian import calcJacobian
import warnings

def IK_velocity (q, v, omega, joint):
    """
    :param q: 1 x 6 vector corresponding to the robot's current configuration.
    :param v: The desired linear velocity in the world frame. If any element is Nan, then that velocity can be anything
    :param omega: The desired angular velocity in the world frame. If any element is Nan, then that velocity can be
                  anything
    :param joint: an integer in [0,6] corresponding to which joint you are tracking
    :return:
    dq - 1 x 6 vector corresponding to the joint velocities. If v and omega
         are infeasible, then dq should minimize the least squares error.

    """
    bodyvelocity=np.array([v[0], v[1], v[2], omega[0], omega[1], omega[2]])
    indexlist=[]
    warnings.filterwarnings("ignore")

    if joint==0 or joint==1:
        return np.array([0, 0, 0, 0, 0, 0])

    jacobianv, jacobianw=calcJacobian(q, joint)
    jacobian=np.vstack((jacobianv, jacobianw))
    jacobianrank=np.linalg.matrix_rank(jacobian)
    combo=np.hstack((jacobian, np.reshape(bodyvelocity, (6, 1))))
    comborank=np.linalg.matrix_rank(combo)
    solexists=0

    if jacobianrank==comborank:
        solexists=1

    for x in range(0, 6):
        if np.isnan(bodyvelocity[x]):
            indexlist.append(x)

    for x in range(0, len(indexlist)):
        bodyvelocity=np.delete(bodyvelocity, indexlist[x]-x, 0)
        jacobian=np.delete(jacobian, indexlist[x]-x, 0)

    pseudoinv=np.linalg.pinv(jacobian)
    dq=pseudoinv.dot(bodyvelocity)

    for x in range(0, 6-dq.size):
        dq=np.hstack((dq, [0]))

    return dq
