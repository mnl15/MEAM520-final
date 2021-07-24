import numpy as np
import math
from calcJacobian import calcJacobian

def FK_velocity (q, dq, joint):
    """
    :param q: 1 x 6 vector corresponding to the robot's current configuration
    :param dq: 1 x 6 vector corresponding to the robot's current joint velocities
    :param joint: an integer in [0,6] corresponding to which joint you are tracking
    :return:
    v     - The resulting linear velocity in the world frame
    omega - The resulting angular velocity in the world frame
    """
    v = np.array([0, 0, 0])
    omega = np.array([0, 0, 0])

    jacobianv, jacobianw=calcJacobian(q, joint)

    if joint==0 or joint==1:
        v=jacobianv.dot(dq[0:1])
        omega=jacobianw.dot(dq[0:1])
    else:
        v=jacobianv.dot(dq[0:joint-1])
        omega=jacobianw.dot(dq[0:joint-1])

    print(jacobianv)

    return v, omega
