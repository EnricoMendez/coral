#!/usr/bin/env python3
from igm import *
from math import *

igm = igm()

U_params = [-167, -575, 311, 0, pi, 0]
U_params = [-109.45,-588.3,217.65,0.152,-pi,0.046]
U_params = [-20.6, -565.6, 221.0, 0, -3.141592653589793, 0]



U = igm.vec2rot(U_params[3:6])
U[:3, 3] = U_params[0:3]

Q = []
home = [pi/2,-pi/2,pi/2,-pi/2,-pi/2,0]
for i in range(1,9):
    q_igm = igm.UR_IGM(U, i)
    solution=[]
    element = q_igm.tolist()
    for number in element:
        j = number[0] * 180 /pi
        solution.append(round(j,4))
    Q.append(solution)
    print(solution)

best = igm.select(Q,home)
print(best)


