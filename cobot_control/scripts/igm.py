import math
import numpy as np

def type2(X, Y, Z):
    theta = [math.nan, math.nan]

    if X == 0 and Y != 0:
        cT = Z / Y
        if cT**2 <= 1:
            theta[0] = math.atan2(math.sqrt(1 - cT**2), cT)
            theta[1] = math.atan2(-math.sqrt(1 - cT**2), cT)

    if Y == 0 and X != 0:
        sT = Z / X
        if sT**2 <= 1:
            theta[0] = math.atan2(sT, math.sqrt(1 - sT**2))
            theta[1] = math.atan2(sT, -math.sqrt(1 - sT**2))

    if Z == 0 and X != 0 and Y != 0:
        theta[0] = math.atan2(-Y, X)
        theta[1] = math.atan2(Y, -X)

    if Z != 0 and X != 0 and Y != 0:
        if X**2 + Y**2 >= Z**2:
            theta[0] = math.atan2((X * Z + Y * math.sqrt(X**2 + Y**2 - Z**2)), (Y * Z - X * math.sqrt(X**2 + Y**2 - Z**2)))
            theta[1] = math.atan2((X * Z - Y * math.sqrt(X**2 + Y**2 - Z**2)), (Y * Z + X * math.sqrt(X**2 + Y**2 - Z**2)))

    return theta

def type3(X1, Y1, Z1, X2, Y2, Z2):
    theta = math.nan
    den = X1 * Y2 - X2 * Y1
    
    if den == 0:
        return theta
    else:
        S = (Z1 * Y2 - Z2 * Y1) / den
        C = (Z2 * X1 - Z1 * X2) / den
        theta = math.atan2(S, C)
    
    return theta


def type8(X, Y, Z1, Z2):
    q = np.zeros((2, 2))

    cos_theta_j = (Z1**2 + Z2**2 - X**2 - Y**2) / (2 * X * Y)
    if abs(cos_theta_j) > 1:
        q[1, 0] = math.nan
        q[1, 1] = math.nan
    else:
        sin_theta_j = math.sqrt(1 - cos_theta_j**2)
        q[1, 0] = math.atan2(sin_theta_j, cos_theta_j)
        q[1, 1] = math.atan2(-sin_theta_j, cos_theta_j)

    for i in range(2):
        B1 = X + Y * math.cos(q[1, i])
        B2 = Y * math.sin(q[1, i])
        BB12 = B1**2 + B2**2
        sin_theta_i = (B1 * Z2 - B2 * Z1) / BB12
        cos_theta_i = (B1 * Z1 + B2 * Z2) / BB12
        q[0, i] = math.atan2(sin_theta_i, cos_theta_i)

    return q

A2 = None
A3 = None
D1 = None
D4 = None
D5 = None
D6 = None

def UR_IGM(U, i):
    global A2, A3, D1, D4, D5, D6
    q_igm = np.zeros((6, 1))
    
    SX = U[0, 0]
    SY = U[1, 0]
    SZ = U[2, 0]
    NX = U[0, 1]
    NY = U[1, 1]
    NZ = U[2, 1]
    AX = U[0, 2]
    AY = U[1, 2]
    AZ = U[2, 2]
    PX = U[0, 3]
    PY = U[1, 3]
    PZ = U[2, 3]
    
    sol = int2bit(i - 1, 3)
    
    X = PX - AX * D6
    Y = AY * D6 - PY
    Z = D4
    q1 = type2(X, Y, Z)
    if sol[0] == 0:
        q_igm[0] = q1[0]
    else:
        q_igm[0] = q1[1]
    C1 = np.cos(q_igm[0])
    S1 = np.sin(q_igm[0])
    
    X = SX * S1 - SY * C1
    Y = NX * S1 - NY * C1
    Z = 0
    q6 = type2(X, Y, Z)
    if sol[1] == 0:
        q_igm[5] = q6[0]
    else:
        q_igm[5] = q6[1]
    C6 = np.cos(q_igm[5])
    S6 = np.sin(q_igm[5])
    
    X1 = 1
    Y1 = 0
    Z1 = (NY * S6 - SY * C6) * C1 + (SX * C6 - NX * S6) * S1
    X2 = 0
    Y2 = 1
    Z2 = AX * S1 - AY * C1
    q5 = type3(X1, Y1, Z1, X2, Y2, Z2)
    q_igm[4] = q5
    C5 = np.cos(q_igm[4])
    S5 = np.sin(q_igm[4])
    
    X = A2
    Y = A3
    Z1 = PX * C1 + PY * S1 - AX * D6 * C1 - AY * D6 * S1 - AX * D4 * C1 * C5 + D5 * NX * C1 * C6 - AY * D4 * C5 * S1 + D5 * NY * C6 * S1 + D5 * SX * C1 * S6 + D5 * SY * S1 * S6 - D4 * SX * C1 * C6 * S5 + D4 * NX * C1 * S5 * S6 - D4 * SY * C6 * S1 * S5 + D4 * NY * S1 * S5 * S6
    Z2 = PZ - D1 - AZ * D6 - AZ * D4 * C5 + D5 * NZ * C6 + D5 * SZ * S6 - D4 * SZ * C6 * S5 + D4 * NZ * S5 * S6
    q23 = type8(X, Y, Z1, Z2)
    if sol[2] == 0:
        q_igm[1:3] = q23[:, 0]
    else:
        q_igm[1:3] = q23[:, 1]
    C2 = np.cos(q_igm[1])
    S2 = np.sin(q_igm[1])
    C3 = np.cos(q_igm[2])
    S3 = np.sin(q_igm[2])
    C23 = np.cos(q_igm[1] + q_igm[2])
    S23 = np.sin(q_igm[1] + q_igm[2])
    
    X1 = 1
    Y1 = 0
    Z1 = AZ * S2 * S3 * S5 - AZ * C2 * C3 * S5 + SZ * C2 * C3 * C5 * C6 - NZ * C2 * C3 * C5 * S6 + AX * C1 * C2 * S3 * S5 + AX * C1 * C3 * S2 * S5 + AY * C2 * S1 * S3 * S5 + AY * C3 * S1 * S2 * S5 - SZ * C5 * C6 * S2 * S3 + NZ * C5 * S2 * S3 * S6 + NY * C2 * C5 * S1 * S3 * S6 + NY * C3 * C5 * S1 * S2 * S6 - SX * C1 * C2 * C5 * C6 * S3 - SX * C1 * C3 * C5 * C6 * S2 + NX * C1 * C2 * C5 * S3 * S6 + NX * C1 * C3 * C5 * S2 * S6 - SY * C2 * C5 * C6 * S1 * S3 - SY * C3 * C5 * C6 * S1 * S2
    X2 = 0
    Y2 = 1
    Z2 = SZ * C2 * C5 * C6 * S3 - AZ * C3 * S2 * S5 - AX * C1 * C2 * C3 * S5 - AY * C2 * C3 * S1 * S5 - AZ * C2 * S3 * S5 + SZ * C3 * C5 * C6 * S2 - NZ * C2 * C5 * S3 * S6 - NZ * C3 * C5 * S2 * S6 + AX * C1 * S2 * S3 * S5 + AY * S1 * S2 * S3 * S5 + NX * C1 * C5 * S2 * S3 * S6 - SY * C5 * C6 * S1 * S2 * S3 + NY * C5 * S1 * S2 * S3 * S6 + SX * C1 * C2 * C3 * C5 * C6 - NX * C1 * C2 * C3 * C5 * S6 + SY * C2 * C3 * C5 * C6 * S1 - NY * C2 * C3 * C5 * S1 * S6 - SX * C1 * C5 * C6 * S2 * S3
    q4 = type3(X1, Y1, Z1, X2, Y2, Z2)
    q_igm[3] = q4

    return q_igm
  
  import numpy as np

def vec2rot(e):
    theta = np.linalg.norm(e)
    vect = e / theta
    
    C = np.cos(theta)
    S = np.sin(theta)
    mid = 1 - C
    
    ux = vect[0]
    uy = vect[1]
    uz = vect[2]
    
    R = np.array([[ux**2*mid + C,   ux*uy*mid - uz*S,   ux*uz*mid + uy*S,   0],
                  [ux*uy*mid + uz*S, uy**2*mid + C,     uy*uz*mid - ux*S,   0],
                  [ux*uz*mid - uy*S, uy*uz*mid + ux*S, uz**2*mid + C,      0],
                  [0,                0,                 0,                  1]])
    
    return R






