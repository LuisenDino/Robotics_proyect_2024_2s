import numpy as np
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH
import spatialmath as sm


class KinematicCalculator():
    def __init__(self):

        self.robot = DHRobot( [
            RevoluteDH(alpha=-math.pi/2, a=0, d=137, offset=0, qlim=[-math.pi, math.pi]),
            RevoluteDH(alpha=0, a=105, d=0, offset=-math.pi/2, qlim=[-math.pi, math.pi]),
            RevoluteDH(alpha=0, a=105, d=0, offset=0, qlim=[-math.pi, math.pi]),
            RevoluteDH(alpha=0, a=95, d=0, offset=0, qlim=[-math.pi, math.pi]),
        ],
        name = 'PhantomX'
        )

        self.robot.tool = np.array([
            [0, 0, 1, 0],
            [-1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ])




    def calcRobotPosition(self, q):
        """
        Calcula la cinemática directa de un robot pincher dado un vector de ángulos de las articulaciones.
        Args:
            q (list): Lista de 4 elementos que representan los ángulos de las articulaciones en grados.
        Returns:
            numpy.ndarray: Matriz de transformación homogénea 4x4 resultante de la multiplicación de las matrices de transformación individuales.
        """

        # self.robot.q = q
        # for i in range(len(q)):
        #     q[i] = math.radians(q[i])   

        
        T = self.robot.fkine(q)
        angle_axis = T.rpy()
        return np.array([T.t[0], T.t[1], T.t[2], angle_axis[2], 0.00], dtype=float)
    
    def calcRobotJoints(self, pose):
        """
        Calcula la cinemática inversa de un robot pincher dado un vector de posición.
        Args:
            pose (list): Lista de 4 elementos que representan la posición del efector final del robot.
        Returns:
            list: Lista de 4 elementos que representan los ángulos de las articulaciones en grados.
        """

        q1 = math.atan2(pose[1], pose[0])
        r = 95*math.cos(math.radians(pose[3]))
        pmx = pose[0] - r*math.cos(q1)
        pmy = pose[1] - r*math.sin(q1)
        if pose[0]*pmx < 0 and pose[1]*pmy < 0:
            rm = -math.sqrt(pmx**2 + pmy**2)
        else:
            rm = math.sqrt(pmx**2 + pmy**2)
        pmz = pose[2] - 95*math.sin(math.radians(pose[3]))
        cos_q3 = (pmx**2 + pmy**2 + (pmz-137)**2 - 22050)/22050
        q3 = math.atan2(math.sqrt(1-cos_q3**2),cos_q3)
        q2 = math.atan2(rm,pmz-137) - math.atan2(105*math.sqrt(1-cos_q3**2),105*(1+cos_q3))
        q4 = 90 - pose[3] - math.degrees(q2) - math.degrees(q3)

        resultado = [q1, q2, q3, math.radians(q4)]

        return resultado


        # q1 = math.atan2(pose[1], pose[0])
        # axis = np.array([math.cos(q1), math.sin(q1), 0])
        # T = sm.SO3.AngVec(pose[3], axis)

        # T_desired = np.array([
        #     [1, 0, 1, pose[0]],
        #     [0, 1, 0, pose[1]],
        #     [0, 0, 1, pose[2]],
        #     [0, 0, 0, 1]
        # ])

        # T_desired[:3, :3] = T

        # # self.robot.plot(self.robot.qz)  # Posición cero del robot

        # return self.robot.ikine_NR(T_desired, q0=[0, 0, 0, 0])

        # q1 = math.atan2(pose[1], pose[0])
        # r = 95*math.cos(pose[3])
        # pmx = pose[0] - r*math.cos(q1)
        # pmy = pose[1] - r*math.sin(q1)
        # if pose[0]*pmx < 0 and pose[1]*pmy < 0:
        #     rm = -math.sqrt(pmx**2 + pmy**2)
        # else:
        #     rm = math.sqrt(pmx**2 + pmy**2)
        # pmz = pose[2] - 95*math.sin(pose[3])
        # cos_q3 = (pmx**2 + pmy**2 + (pmz-137)**2 - 22050)/22050
        # q3 = math.atan2(math.sqrt(1-cos_q3**2),cos_q3)
        # q2 = math.atan2(rm,pmz-137) - math.atan2(105*math.sqrt(1-cos_q3**2),105*(1+cos_q3))
        # q4 = 90 - pose[3] - q2 - q3

        # resultado = [q1, q2, q3, q4]

        # return resultado




        l1 = 137
        l2 = 105
        l3 = 105
        l4 = 95

        x = pose[0]
        y = pose[1]
        z = pose[2]
        gamma = pose[3]
        q1 = math.atan2(y,x)
        r = math.sqrt(x**2 + y**2) - l4*math.cos(gamma)
        z_eff = z - l1 - l4*math.sin(gamma)
        cos_q3 = (r**2 + z_eff**2 - l2**2 - l3**2)/(2*l2*l3)
        print(cos_q3)
        q3 = math.atan2(-math.sqrt(1-cos_q3**2),cos_q3)
        alpha = math.atan2(z_eff,r)
        beta = math.atan2(l3*math.sin(q3),l2+l3*math.cos(q3))
        q2 = math.pi/2 - alpha - beta
        q4 = math.pi/2-(q3 + q2) - gamma 

        resultado = [q1, q2, q3, q4]

        return resultado

        
