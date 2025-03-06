import numpy as np
import math
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH
import spatialmath as sm


class KinematicCalculator():
    def __init__(self):
        self.gripper = 0

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

    def calc_robot_position(self, q):
        """
        Calcula la cinemática directa de un robot pincher dado un vector de ángulos de las articulaciones.
        Args:
            q (list): Lista de 4 elementos que representan los ángulos de las articulaciones en grados.
        Returns:
            numpy.ndarray: Matriz de transformación homogénea 4x4 resultante de la multiplicación de las matrices de transformación individuales.
        """

        # self.robot.q = q
        for i in range(len(q)):
            q[i] = math.radians(q[i])   

        
        T = self.robot.fkine(q)

        gamma = 90 - (math.degrees(q[3])+math.degrees(q[2]) +  math.degrees(q[1]))

        return np.array([T.t[0], T.t[1], T.t[2], gamma, 0.00], dtype=float)
    
    def calc_robot_joints(self, pose):
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
        if pose[0]*pmx <= 0 and pose[1]*pmy <= 0:
            rm = -math.sqrt(pmx**2 + pmy**2)
        else:
            rm = math.sqrt(pmx**2 + pmy**2)
        pmz = pose[2] - 95*math.sin(math.radians(pose[3]))
        cos_q3 = (pmx**2 + pmy**2 + (pmz-137)**2 - 22050)/22050
        q3 = math.atan2(math.sqrt(1-cos_q3**2),cos_q3)
        q2 = math.atan2(rm,pmz-137) - math.atan2(105*math.sqrt(1-cos_q3**2),105*(1+cos_q3))
        q4 = 90 - pose[3] - math.degrees(q2) - math.degrees(q3)

        resultado = [q1, q2, q3, math.radians(q4)]

        for i in range(len(resultado)):
            resultado[i] = math.degrees(resultado[i])

        return resultado

    def get_gripper_position(self, data):
        """
        Calcula la apertura o cierre del gripper
        Args:
            data (float): Posición del gripper en porcentaje.
        Returns:
            int: 0 si el gripper está cerrado, 1 si está abierto.
        """
        if data > 0.7:
            self.gripper = 1
        if data < -0.7:
            self.gripper = 0
        return self.gripper
