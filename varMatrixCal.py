from sympy import *
from pandas import *
import numpy as np
import numpy.linalg as la

def trsp(x, y, z):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]])
# Rotation Matrix 3x3 about x --> y --> z

def rot(axis, val):
    val = (np.pi/180)*val
    [s1, c1] = sin(val), cos(val)
    if axis == 'x':
        R = [[1, 0, 0],
            [0, c1, -s1],
            [0, s1, c1]]
    if axis == 'y':
        R = [[c1, 0, s1],
            [0, 1, 0],
            [-s1, 0, c1]]
    if axis == 'z':
        R = [[c1, -s1, 0],
            [s1, c1, 0],
            [0, 0, 1]]
    return rndX(R)
# Rotation Matrix 4x4 from 3x3


def rot4(R):
    R = np.concatenate((R, [[0], [0], [0]]), axis=1)
    R = np.concatenate((R, [[0, 0, 0, 1]]), axis=0)
    return R


def rndX(A_input):
    A = A_input
    for i, element in enumerate(A):
        for j, data in enumerate(element):
            #print('PRE: ',data)
            for subData in preorder_traversal(data):
                if isinstance(subData, Float):
                    data = data.subs(subData, round(subData, 3))
                    try:
                        A[i][j]=round(data, 3)
                    except:
                        A[i][j]=data
    return A

def trim(A_input):
    A = A_input
    for i, element in enumerate(A):
        for j, data in enumerate(element):
            data = str(data).replace('in(0.017','(')
            data = str(data).replace('os(0.017','(')
            data = str(data).replace('-1.0*','-')
            data = str(data).replace('1.0*','')
            data = str(data).replace('*1.0','')
            data = str(data).replace('**','^')
            data = str(data).replace('*','')
            data = str(data).replace('.0','')
            data = str(data).replace('(Z1)','1')
            data = str(data).replace('(Z2)','2')
            data = str(data).replace('(Z3)','3')
            data = str(data).replace('(Z4)','4')
            data = str(data).replace('(Z5)','5')
            data = str(data).replace('(Z6)','6')
            #print (data)
            A[i][j]=data
    print(DataFrame(A))


'''
# Find T_AB
T_AC = trsp(5, 4, 1)
T_CD = rot4(rot(x, 0, 0))
T_DE = trsp(2, 0, 2)
T_EF = rot4(rot(0, 60, 0))
T_FB = trsp(-2, 0, -2)
'''

# Manipulator Kinematics


def trsm(i, alp_1, zta_i, a, d):
    i = i-1
    alp_1[i] = (np.pi/180)*alp_1[i]
    zta_i[i] = (np.pi/180)*zta_i[i]
    ca_1 = cos(alp_1[i])
    cz_i = cos(zta_i[i])
    sa_1 = sin(alp_1[i])
    sz_i = sin(zta_i[i])
    T = [[cz_i, -sz_i, 0, a[i]],
         [sz_i*ca_1, cz_i*ca_1, -sa_1, -sa_1*d[i]],
         [sz_i*sa_1, cz_i*sa_1, ca_1, ca_1*d[i]],
         [0, 0, 0, 1]]
    return rndX(np.array(T))

#------------------------------------------------------------------------
'''
# Fig.3.21
alp_i1 = [0, -90, 0, -90, 90, -90]
zta_i = [Z1, Z2, Z3, Z4, Z5, Z6]
a_i1 = [0, 0, a2, a3, 0, 0]
d_i = [0, 0, d3, d4, 0, 0]
T = [1,2,3,4,5,6]

# MCE341-Fin2-57-1.1
alp_i1 = [0, 90, 0]
zta_i = [Z1, Z2, Z3]
a_i1 = [L1, 0, 0]
d_i = [0, L2, L3]
T = [1,2,3]
for i, element in enumerate(T):
    T[i-1] = trsm(i, alp_i1, zta_i, a_i1, d_i)
res = la.multi_dot(T)
#print(trim(res))
res = la.multi_dot([rot(0,0,z), rot(0,y,0), rot(x,0,0)])
res = la.multi_dot([rot(x,0,0),rot(0,y,0)])
print(trim(rndX(res)))
'''

# round
rp = 3
# define all symbols
a1, a2, a3, a4, a5, a6 = symbols('a(1:7)')
d1, d2, d3, d4, d5, d6 = symbols('d(1:7)')
L1, L2, L3, L4, L5, L6 = symbols('L(1:7)')
W1, W2, W3, W4, W5, W6 = symbols('W(1:7)')
Wd1, Wd2, Wd3, Wd4, Wd5, Wd6 = symbols('Wd(1:7)')
Z1, Z2, Z3, Z4, Z5, Z6 = symbols('Z(1:7)')
x, y, z = symbols('x y z')

R01 = rot('z', Z1)
R11 = rot('x', 90)
R12 = rot('z', Z2)
R02 = la.multi_dot([R01, R11, R12])
R23 = rot('z', Z3)
R03 = la.multi_dot([R02, R23])

P0_01 = [[L1], [0], [0]]
P1_12 = [[0], [-L2], [0]]
P2_23 = [[0], [0], [L3]]
P0_12 = la.multi_dot([R01, P1_12])
P0_23 = la.multi_dot([R02, P2_23])
P0_33 = [[0], [0], [0]]

W1_m1 = [[0], [0], [W1]]
W2_m2 = [[0], [0], [W2]]
W3_m3 = [[0], [0], [W3]]
W00 = [[0], [0], [0]]
W01 = np.dot(R01, W1_m1)
W02 = np.dot(R02, W2_m2)
W03 = np.dot(R03, W3_m3)

V01 = np.cross(W01, np.add(P0_12, P0_23), axisa=0, axisb=0, axisc=0)
V02 = np.cross(W02, P0_23, axisa=0, axisb=0, axisc=0)
V03 = np.cross(W03, P0_33, axisa=0, axisb=0, axisc=0)

W1_m1 = [[0], [0], [Wd1]]
W2_m2 = [[0], [0], [Wd2]]
W3_m3 = [[0], [0], [Wd3]]
Wd_01 = np.dot(R01, Wd1_m1)
Wd_02 = np.dot(R02, Wd2_m2)
Wd_03 = np.dot(R03, Wd3_m3)
Wd_01 = np.add()

print('-------------------')
trim(V01)
print('-------------------')
trim(V02)
print('-------------------')
trim(V03)
print('-------------------')
trim(W01)
print('-------------------')
trim(W02)
print('-------------------')
trim(W03)
print('-------------------')
trim(P0_23)
#print(la.dot(a1, a2))