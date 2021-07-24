import numpy as np
import math

class calculateIK():

    def __init__(self):
        """
        This is the dimension of the Lynx Robot stated as global variable

        """
        # Lynx ADL5 constants in mm
        self.d1 = 76.2                      # Distance between joint 1 and joint 2
        self.a2 = 146.05                    # Distance between joint 2 and joint 3
        self.a3 = 187.325                   # Distance between joint 3 and joint 4
        self.d4 = 34                        # Distance between joint 4 and joint 5
        self.d5 = 68                        # Distance between joint 4 and end effector

        # Joint limits
        self.lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
        self.upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)

    def inverse(self, T0e):
        """
        INPUT:
        T - 4 x 4 homogeneous transformation matrix, representing
           the end effector frame expressed in the base (0) frame
           (position in mm)

        OUTPUT:
        q - a n x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad)
           which are required for the Lynx robot to reach the given
           transformation matrix T. Each row represents a single
           solution to the IK problem. If the transform is
           infeasible, q should be all zeros.
        isPos - a boolean set to true if the provided
             transformation T is achievable by the Lynx robot as given,
             ignoring joint limits
        """
        isPos = 1
        flip=0
        q = np.zeros((1, 5))
        # Your code starts from here
        solset=np.zeros((1,7))
        d1=self.d1
        a2=self.a2
        a3=self.a3
        d4=self.d4
        d5=self.d5

        infsol=0
        remap=0

        lowerLim=self.lowerLim
        upperLim=self.upperLim

        R=np.zeros((3,3))

        R=T0e[0:3, 0:3]
        ox=T0e[0, 3]
        oy=T0e[1, 3]
        oz=T0e[2, 3]

        xc=ox-(d5*R[0, 2])
        yc=oy-(d5*R[1, 2])
        zc=oz-(d5*R[2, 2])

        if math.sqrt(xc**2+yc**2+((zc-76.2)**2))>(d1+a2+a3):
            isPos=0
            return np.array([0, 0, 0, 0, 0, 0]), 7, 0

        if yc==0 and xc==0:
            q1a=0
            infsol=1
            q1b=math.pi
        else:
            p=np.array([ox, oy, oz])
            porig=np.array([0, 0, 0])
            porig2=np.array([0, 0, 1])
            z0e=[R[0, 2], R[1, 2], R[2, 2]]
            y0e=[R[0, 1], R[1, 1], R[2, 1]]


            normal=np.cross(p-porig, p-porig2)
            normalmag=math.sqrt(normal[0]**2+normal[1]**2+normal[2]**2)
            normalunit=normal/normalmag

            projz1=z0e-((np.matmul(z0e, normalunit))*normalunit)
            projz2=-projz1
            zdistance1=round(abs((z0e[0]-projz1[0])+(z0e[1]-projz1[1])+(z0e[2]-projz1[2])), 3)
            zdistance2=round(abs((z0e[0]-projz2[0])+(z0e[1]-projz2[1])+(z0e[2]-projz2[2])), 3)
            if zdistance1!=0:
                remap=1

                if zdistance1<zdistance2:
                    projz=projz1
                else:
                    projz=projz2

                projy1=normalunit
                projy2=-normalunit
                ydistance1=abs((y0e[0]-projy1[0])+(y0e[1]-projy1[1])+(y0e[2]-projy1[2]))
                ydistance2=abs((y0e[0]-projy2[0])+(y0e[1]-projy2[1])+(y0e[2]-projy2[2]))
                if ydistance1<ydistance2:
                    projy=projy1
                else:
                    projy=projy2

                projy=np.round(projy, 3)
                projz=np.round(projz, 3)
                projx=np.round(np.cross(projy, projz), 3)
                R[:, 0]=projx
                R[:, 1]=projy
                R[:, 2]=projz
                isPos=0
                xc=ox-(d5*R[0, 2])
                yc=oy-(d5*R[1, 2])
                zc=oz-(d5*R[2, 2])

        if yc==0 and xc==0:
            q1a=0
            infsol=1
            q1b=math.pi
        else:
            q1a=math.atan2(yc, xc)
            if q1a>0:
                q1b=q1a-math.pi
            else:
                q1b=q1a+math.pi

        D=((xc**2)+(yc**2)+(zc-d1)**2-(a2**2)-(a3**2))/(2*a2*a3)
        D=round(D, 3)
        if D>1:
            return np.array([0, 0, 0, 0, 0, 0]), 7, 0
        q3posa=(-math.pi/2)-math.atan2(math.sqrt(1-D**2), D)
        q3nega=(-math.pi/2)-math.atan2(-math.sqrt(1-D**2), D)
        q2posa=(math.pi/2)-math.atan2(zc-d1, math.sqrt(xc**2+yc**2))+math.atan2(a3*math.sin((-math.pi/2)-q3posa), a2+(a3*math.cos((-math.pi/2)-q3posa)))
        q2nega=(math.pi/2)-math.atan2(zc-d1, math.sqrt(xc**2+yc**2))+math.atan2(a3*math.sin((-math.pi/2)-q3nega), a2+(a3*math.cos((-math.pi/2)-q3nega)))

        q2posb=-q2posa
        q2negb=-q2nega

        if q3posa>0:
            q3posb=-q3posa+math.pi
        else:
            q3posb=-q3posa-math.pi

        if q3nega>0:
            q3negb=-q3nega+math.pi
        else:
            q3negb=-q3nega-math.pi

        check=0

        sols=self.rotmax(q1a, q2posa, q3posa, R)
        solset=[q1a, q2posa, q3posa, sols[0], sols[1], infsol]
        sols=self.rotmax(q1a, q2nega, q3nega, R)
        solset=np.vstack((solset, [q1a, q2nega, q3nega, sols[0], sols[1], infsol]))
        sols=self.rotmax(q1b, q2posb, q3posb, R)
        solset=np.vstack((solset, [q1b, q2posb, q3posb, sols[0], sols[1], infsol]))
        sols=self.rotmax(q1b, q2negb, q3negb, R)
        solset=np.vstack((solset, [q1b, q2negb, q3negb, sols[0], sols[1], infsol]))

        i=0
        for x in range(0, 4):
            for y in range(0, 5):
                if solset[x, y]>math.pi:
                    for b in range(0, 10):
                        solset[x, y]=solset[x,y]-2*math.pi
                        if solset[x,y]<=math.pi:
                            break
                elif solset[x, y]<-math.pi:
                    for b in range(0, 10):
                        solset[x, y]=solset[x,y]+2*math.pi
                        if solset[x,y]>=-math.pi:
                            break
                else:
                    continue

        i=0
        for x in range(0,4):
            check=0
            for y in range(0, 5):
                if solset[x, y]<lowerLim[0, y] or solset[x, y]>upperLim[0,y]:
                    if y==4:
                        if solset[x,y]>0:
                            solset[x,y]=solset[x, y]-math.pi
                            flip=1
                        else:
                            solset[x,y]=solset[x, y]+math.pi
                            flip=1
                    else:
                        check=1
                        break
            if check==0:
                if solset[x, 5]==1:
                    q[0,:]=[float("NaN"), solset[x, 1], solset[x, 2], solset[x, 3], float("NaN")]
                else:
                    if i>=1:
                        if np.array_equal(q[0, :], solset[x, 0:5])==0:
                            q=np.vstack((q, solset[x, 0:5]))
                        else:
                            continue
                    else:
                        q[i, :]=solset[x, 0:5]
                i=i+1
                sols=1

        q=np.round(q, 3)
        if np.count_nonzero(q)==0 and sols!=1:
            q=np.empty
            isPos=7


        #print(solset)
        #print(q)
        #print(isPos)
        # Your code ends here

        return q, isPos, flip

    def rotmax(self, q1, q2, q3, R):
        q1=q1
        q2=q2-math.pi/2
        q3=q3+math.pi/2

        R03=np.array([[math.cos(q1)*math.cos(q2+q3), -math.cos(q1)*math.sin(q2+q3), -math.sin(q1)],
                     [math.sin(q1)*math.cos(q2+q3), -math.sin(q1)*math.sin(q2+q3), math.cos(q1)],
                     [-math.sin(q2+q3), -math.cos(q2+q3), 0]])
        R3e=np.zeros((3, 3))
        R3e=np.round(np.matmul(np.linalg.inv(R03),R), 3)

        q4=math.atan2(R3e[0,2], -R3e[1, 2])-math.pi/2
        q5=math.atan2(R3e[2,0], R3e[2,1])-math.pi

        return q4, q5
