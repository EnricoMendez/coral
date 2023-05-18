from igm import *

igm = igm()

U_params = [-167, -575, 311, 0, math.pi, 0]
U = igm.vec2rot(U_params[3:6])
U[:3, 3] = U_params[0:3]

Q = np.zeros((6, 8))

for i in range(1,9):
    print(i)
    q_igm = igm.UR_IGM(U, i)
    print(q_igm)
