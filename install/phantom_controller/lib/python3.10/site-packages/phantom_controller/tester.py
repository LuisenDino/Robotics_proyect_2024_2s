from KinematicCalculator import KinematicCalculator
import numpy as np
import math


pos = np.array( [55 , 65 ,185 ,  math.radians(-25)], dtype=float)

q = np.array([0, 0, 0, 0], dtype=float)

kin = KinematicCalculator()
q = kin.calcRobotJoints(pos)

print(q)
print(kin.calcRobotPosition(q))
