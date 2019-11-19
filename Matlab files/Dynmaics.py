from sympy import *
import numpy as np

# Known constants
g, m1, m2, m3, mp, L1, L2, Lc1, Lc2, Lc3, Lcp, t = symbols(
    'g m1 m2 m3 mp L1 L2 Lc1 Lc2 Lc3 Lcp t', real=true)

I1 =Matrix([
    [   0.0001130151, -0.00000860914,  0.00000048865], 
    [ -0.00000860914,  0.00008504653, -0.00000210564], 
    [  0.00000048865, -0.00000210564,   0.0001180217]]
)

I2 = Matrix([
[  0.00006763106,    0.000003336, -0.00000058233],
[    0.000003336,   0.0006261946, -0.00000477121],
[ -0.00000058233, -0.00000477121,    0.000635118]])

I3 = Matrix([
[ 0.00009477987, 0.00000560523, 0.00000272008], 
[ 0.00000560523,  0.0003202224, 0.00000034515], 
[ 0.00000272008, 0.00000034515,  0.0002467779]
])

# Theta as function of t
t1 = Function('t1', real=True)(t)
t2 = Function('t2', real=True)(t)
t3 = Function('t3', real=True)(t)

dt1 = diff(t1, t)
#dt1 = Function('dt1', real=True)(t)
dt2 = diff(t2, t)
#dt2 = Function('dt2', real=True)(t)
dt3 = diff(t3, t)
#dt3 = Function('dt3', real=True)(t)

ddt1 = diff(dt1, t)
#ddt1 = Function('ddt1', real=True)(t)
ddt2 = diff(dt2, t)
#ddt2 = Function('ddt2', real=True)(t)
ddt3 = diff(dt3, t)
#ddt3 = Function('ddt3', real=True)(t)

# HRotational matrix
def rotx(theta):
    M = Matrix([[1, 0, 0],
                [0, cos(theta), -sin(theta)],
                [0, sin(theta), cos(theta)]])
    return M


def rotz(theta):
    M = Matrix([[cos(theta), -sin(theta), 0],
                [sin(theta), cos(theta), 0],
                [0, 0, 1]])
    return M


R2 = rotz(t1)*rotx(pi/2)*rotz(t2)
R3 = rotz(t1)*rotx(pi/2)*rotz(t2+t3)

# Z-axis of joint (Z1 is also global Z)
Z1 = Matrix([[0], [0], [1]])
Z2 = R2*Z1
Z3 = R3*Z1

# Angular velociy
W1 = Z1*dt1  # W1dot = W1(1)^2+W1(2)^2+W1(3)^2
W2 = W1+Z2*dt2  # W2dot = W2(1)^2+W2(2)^2+W2(3)^2
W3 = W2+Z3*dt3  # W3dot = W3(1)^2+W3(2)^2+W3(3)^2

# Vector from previous joint to center of mass
Sc1 = Matrix([[0], [0], [Lc1]])
Sc2 = R2*Matrix([[Lc2], [0], [0]])
Sc3 = R3*Matrix([[Lc3], [0], [0]])
Scp = R3*Matrix([[Lcp], [0], [0]])

# Vector from previous joint to next joint
S1 = Matrix([[0], [0], [L1]])
S2 = R2*Matrix([[L2], [0], [0]])

# Joint velocity
v1 = Matrix([[0], [0], [0]])
v2 = v1+W1.cross(S1)
v3 = v2+W2.cross(S2)

# Velocity in center of mass
vc2 = v2+W2.cross(Sc2)  # vc2dot = vc2(1)^2+vc2(2)^2+vc2(3)
vc3 = v3+W3.cross(Sc3)  # vc3dot = vc3(1)^2+vc3(2)^2+vc3(3)
vcp = v3+W3.cross(Scp)  # vcpdot = vcp(1)^2+vcp(2)^2+vcp(3)

# Height at center of mass
h1 = Lc1
h2 = L1+Lc2*sin(t2)
h3 = L1+L2*sin(t2)+Lc3*sin(t2+t3)
hp = L1+L2*sin(t2)+Lcp*sin(t2+t3)


# Kinetic and potential energy
# Ti = (1/2)*mi*dot(vci,vci) + (1/2)*dot(Wi,I*Wi)

T1 = (1/2)*W1.dot(I1*W1)
V1 = m1*g*h1

T2 = (1/2)*m2*vc2.dot(vc2) + (1/2)*W2.dot((R2*I2*R2.T)*W2)
V2 = m2*g*h2

T3 = (1/2)*m3*vc3.dot(vc3)+(1/2)*W3.dot((R3*I3*R3.T)*W3)
V3 = m3*g*h3

# Payload
Tp = (1/2)*mp*vcp.dot(vcp)
Vp = mp*g*hp

# Lagrangian equation
L = T1-V1+T2-V2+T3-V3+Tp-Vp

tau1 = diff(diff(L, dt1), t) - diff(L, t1)
tau2 = diff(diff(L, dt2), t) - diff(L, t2)
tau3 = diff(diff(L, dt3), t) - diff(L, t3)

dt1 = Function('dt1', real=True)(t)
dt2 = Function('dt2', real=True)(t)
dt3 = Function('dt3', real=True)(t)

#ddt1 = Function('ddt1', real=True)(t)
#ddt2 = Function('ddt2', real=True)(t)
#ddt3 = Function('ddt3', real=True)(t)
 
#dt1 = 0
#dt2 = 0
#dt3 = 0

ddt1 = 0
ddt2 = 0
ddt3 = 0

subs_subs_tau = [
(diff(t1,t), dt1), (diff(t2,t), dt2 ), (diff(t3,t), dt3 ),
(diff(dt1,t), ddt1 ), (diff(dt2,t), ddt2 ), (diff(dt3,t), ddt3 )
]

tau1 = tau1.subs(subs_subs_tau)
tau2 = tau2.subs(subs_subs_tau)
tau3 = tau3.subs(subs_subs_tau)

G1 = 0
G2 = 1.0*L2*g*m3*cos(t2) + 1.0*L2*g*mp*cos(t2) + 1.0*Lc2*g*m2*cos(t2) + 1.0*Lc3*g*m3*cos(t2 + t3) + 1.0*Lcp*g*mp*cos(t2 + t3)
G3 = 1.0*Lc3*g*m3*cos(t2 + t3) + 1.0*Lcp*g*mp*cos(t2 + t3)

tau1 = tau1 - G1
tau2 = tau2 - G2
tau3 = tau3 - G3

print("Tau1:")
print(ccode(simplify(tau1)))
print("Tau2:")
print(ccode(simplify(tau2)))
print("Tau3:")
print(ccode(simplify(tau3)))




