import numpy as np
from numpy import linalg
import math
from math import pi as pi
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
import cmath
from pprint import pprint
import copy
import time
import lab_exercises.lab_palopoli.params as conf
from scipy.optimize import fsolve
from pprint import pprint

np.set_printoptions(precision=6, suppress=True)

global mat
mat = np.matrix
zero_array = np.array([0, 0, 0, 0, 0, 0])

# ****** Coefficients ******
global d1, a2, a3, d4, d5, d6, gripper_lenght
if conf.robot_params['ur5']['soft_gripper']:
    gripper_lenght = 0.1475
else:
    gripper_lenght = 0.17

d1 = 0.163
a2 = -0.42500
a3 = -0.39225
d4 = 0.134
d5 = 0.100
d6 = 0.100 + gripper_lenght

global d, a, alph
#       0 -> 1  -> 2 ->3 ->4  -> 5   -> 6
d = mat([d1, 0, 0, d4, d5, d6])
a = mat([0, 0, a2, a3, 0, 0])
alph = mat([pi, pi / 2, 0, 0, pi / 2, -pi / 2])

def rotX(roll):
    return mat([[1, 0, 0], 
                [0, cos(roll), -sin(roll)], 
                [0, sin(roll),  cos(roll)]])

def rotY(pitch):
    return mat([[ cos(pitch), 0, sin(pitch)], 
                [0, 1, 0], 
                [-sin(pitch), 0, cos(pitch)]])

def rotZ(yaw):
    return mat([[cos(yaw), -sin(yaw), 0], 
                [sin(yaw),  cos(yaw), 0], 
                [0, 0, 1]])

# ----------- DIRECT KINEMATICS ------------------------
def T01mtrans(theta1):
    # ---------------------  0->1   ------------------------------------
    # matrice di rotazione iniziale di pi attorno all'asse x0 e displacement di d1 sull'asse z
    rot_inizialeX = mat([[1, 0, 0, 0],
                         [0, cos(alph[0, 0]), -sin(alph[0, 0]), 0],
                         [0, sin(alph[0, 0]), cos(alph[0, 0]), 0],
                         [0, 0, 0, 1]])
    # traslazione sull'asse z1 di d1
    trasl_Z = mat([[1, 0, 0, 0],
                   [0, 1, 0, 0],
                   [0, 0, 1, d[0, 0]],
                   [0, 0, 0, 1]])
    # generica rotazione di t1 attorno all'asse z1
    rotZ1 = mat([[cos(theta1), -sin(theta1), 0, 0],
                 [sin(theta1), cos(theta1), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
    # trasformazione finale dal frame 0 al frame 1
    return rot_inizialeX * trasl_Z * rotZ1

def T12mtrans(theta2):
    # ---------------------  1->2   ------------------------------------
    # rotazione attorno all'asse x1 per configurare il frame 2
    rotX1 = mat([[1, 0, 0, 0],
                 [0, cos(alph[0, 1]), -sin(alph[0, 1]), 0],
                 [0, sin(alph[0, 1]), cos(alph[0, 1]), 0],
                 [0, 0, 0, 1]])
    rotZ2 = mat([[cos(theta2), -sin(theta2), 0, 0],
                 [sin(theta2), cos(theta2), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
    # trasformazione finale dal frame 1 al frame 2
    return rotX1 * rotZ2

def T23mtrans(theta3):
    # ---------------------  2->3   ------------------------------------
    trasl_x3 = mat([[1, 0, 0, a[0, 2]],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    rotZ3 = mat([[cos(theta3), -sin(theta3), 0, 0],
                 [sin(theta3), cos(theta3), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
    # trasformazione finale dal frame 2 al frame 3
    return trasl_x3 * rotZ3

def T34mtrans(theta4):
    # ---------------------  3->4   ------------------------------------
    trasl_x_z = mat([[1, 0, 0, a[0, 3]],
                     [0, 1, 0, 0],
                     [0, 0, 1, d[0, 3]],
                     [0, 0, 0, 1]])
    rotZ4 = mat([[cos(theta4), -sin(theta4), 0, 0],
                 [sin(theta4), cos(theta4), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
    # trasformazione finale dal frame 3 al frame 4
    return trasl_x_z * rotZ4

def T45mtrans(theta5):
    # ---------------------  4->5   ------------------------------------
    # rotazione attorno all'asse x4 per configurare il frame 5
    rotX4 = mat([[1, 0, 0, 0],
                 [0, cos(alph[0, 4]), -sin(alph[0, 4]), 0],
                 [0, sin(alph[0, 4]), cos(alph[0, 4]), 0],
                 [0, 0, 0, 1]])
    trasl_z4 = mat([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, d[0, 4]],
                    [0, 0, 0, 1]])
    rotZ5 = mat([[cos(theta5), -sin(theta5), 0, 0],
                 [sin(theta5), cos(theta5), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
    # trasformazione finale dal frame 4 al frame 5
    return rotX4 * trasl_z4 * rotZ5

def T56mtrans(theta6):
    # ---------------------  5->6   ------------------------------------
    # rotazione attorno all'asse x4 per configurare il frame 5
    rotX5 = mat([[1, 0, 0, 0],
                 [0, cos(alph[0, 5]), -sin(alph[0, 5]), 0],
                 [0, sin(alph[0, 5]), cos(alph[0, 5]), 0],
                 [0, 0, 0, 1]])
    trasl_z5 = mat([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, d[0, 5]],
                    [0, 0, 0, 1]])
    rotZ6 = mat([[cos(theta6), -sin(theta6), 0, 0],
                 [sin(theta6), cos(theta6), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
    # trasformazione finale dal frame 5 al frame 6
    return rotX5 * trasl_z5 * rotZ6

def T03mtrans(jstate):
    T01 = T01mtrans(jstate[0])
    T02 = T01 * T12mtrans(jstate[1])
    T03 = T02 * T23mtrans(jstate[2])

    return T03

def T04mtrans(jstate):
    T06 = direct_kin(jstate)
    T04 = T01mtrans(jstate[0]) * T12mtrans(jstate[1]) * T23mtrans(jstate[2]) * T34mtrans(0)
    T46 = np.linalg.inv(T04) * T06

    return T46

def direct_kin(jstate):
    T06 = T01mtrans(jstate[0]) * T12mtrans(jstate[1]) * T23mtrans(jstate[2]) * T34mtrans(jstate[3]) * T45mtrans(
        jstate[4]) * T56mtrans(jstate[5])
    return T06


# ----------- INVERSE KINEMATICS ------------------------
def T01trans(th1):
    return mat([[cos(th1), -sin(th1), 0, 0],
                [sin(th1), cos(th1), 0, 0],
                [0, 0, 1, d[0, 0]],
                [0, 0, 0, 1]])

def T12trans(th2):
    return mat([[cos(th2), -sin(th2), 0, 0],
                [0, 0, -1, 0],
                [sin(th2), cos(th2), 0, 0],
                [0, 0, 0, 1]])

def T23trans(th3):
    return mat([[cos(th3), -sin(th3), 0, a[0, 2]],
                [sin(th3), cos(th3), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

def T34trans(th4):
    return mat([[cos(th4), -sin(th4), 0, a[0, 3]],
                [sin(th4), cos(th4), 0, 0],
                [0, 0, 1, d[0, 3]],
                [0, 0, 0, 1]])

def T45trans(th5):
    return mat([[cos(th5), -sin(th5), 0, 0],
                [0, 0, -1, -d[0, 4]],
                [sin(th5), cos(th5), 0, 0],
                [0, 0, 0, 1]])

def T56trans(th6):
    return mat([[cos(th6), -sin(th6), 0, 0],
                [0, 0, 1, d[0, 5]],
                [-sin(th6), -cos(th6), 0, 0],
                [0, 0, 0, 1]])

def inverse_kin(des_pos_usd, des_rotm_usd, actual_angles=zero_array):
    # ricevuta la posizione "des_pos_upsidedown", cioè la posizione desiderata nella configurazione con il braccio "a testa in giù",
    # la trasformo in una posizione in cui il braccio si sviluppa in verticale (grazie alla trasformazione di rotazione attorno all'asse x)
    # in modo da essere in grado di utilizzare la IK sviluppata dal prof.

    th = mat(np.zeros((6, 8)))

    des_pose_upsidedown = np.eye(4)
    des_pose_upsidedown[0:3, 0:3] = des_rotm_usd
    des_pose_upsidedown[0:3, 3] = des_pos_usd.T

    trans_frame_to_normal = mat([[1, 0, 0, 0],
                                 [0, cos(alph[0, 0]), -sin(alph[0, 0]), 0],
                                 [0, sin(alph[0, 0]), cos(alph[0, 0]), 0],
                                 [0, 0, 0, 1]])

    des_pos = trans_frame_to_normal * des_pose_upsidedown
    p_des = des_pos[0:3, 3:4].T

    # *********** theta1 ***********
    p05 = des_pos * mat([0, 0, -d[0, 5], 1]).T

    th1_1 = np.round(atan2(p05[1], p05[0]) + np.real(cmath.acos(d[0, 3] / np.hypot(p05[1], p05[0]))) + pi / 2, 4)
    th1_2 = np.round(atan2(p05[1], p05[0]) - np.real(cmath.acos(d[0, 3] / np.hypot(p05[1], p05[0]))) + pi / 2, 4)

    # *********** theta5 ***********
    th5_1 = np.round( np.real(cmath.acos((p_des[0, 0] * sin(th1_1) - p_des[0, 1] * cos(th1_1) - d[0, 3]) / d[0, 5])), 4)
    th5_2 = np.round(-np.real(cmath.acos((p_des[0, 0] * sin(th1_1) - p_des[0, 1] * cos(th1_1) - d[0, 3]) / d[0, 5])), 4)
    th5_3 = np.round( np.real(cmath.acos((p_des[0, 0] * sin(th1_2) - p_des[0, 1] * cos(th1_2) - d[0, 3]) / d[0, 5])), 4)
    th5_4 = np.round(-np.real(cmath.acos((p_des[0, 0] * sin(th1_2) - p_des[0, 1] * cos(th1_2) - d[0, 3]) / d[0, 5])), 4)

    # *********** theta6 ***********
    T60 = np.linalg.inv(des_pos)
    Xhat = T60[0:3, 0]
    Yhat = T60[0:3, 1]

    th6_1 = 0
    th6_2 = 0
    th6_3 = 0
    th6_4 = 0

    if th5_1 != 0:
        th6_1 = np.round(np.real(atan2((-Xhat[1] * sin(th1_1) + Yhat[1] * cos(th1_1)) / sin(th5_1),
                                       (Xhat[0] * sin(th1_1) - Yhat[0] * cos(th1_1)) / sin(th5_1))), 4)
    if th5_2 != 0:
        th6_2 = np.round(np.real(atan2((-Xhat[1] * sin(th1_1) + Yhat[1] * cos(th1_1)) / sin(th5_2),
                                       (Xhat[0] * sin(th1_1) - Yhat[0] * cos(th1_1)) / sin(th5_2))), 4)
    if th5_3 != 0:
        th6_3 = np.round(np.real(atan2((-Xhat[1] * sin(th1_2) + Yhat[1] * cos(th1_2)) / sin(th5_3),
                                       (Xhat[0] * sin(th1_2) - Yhat[0] * cos(th1_2)) / sin(th5_3))), 4)
    if th5_4 != 0:
        th6_4 = np.round(np.real(atan2((-Xhat[1] * sin(th1_2) + Yhat[1] * cos(th1_2)) / sin(th5_4),
                                       (Xhat[0] * sin(th1_2) - Yhat[0] * cos(th1_2)) / sin(th5_4))), 4)

    # *********** theta3 ***********
    T14 = np.linalg.inv(T01trans(th1_1)) * des_pos * np.linalg.inv(T56trans(th6_1)) * np.linalg.inv(T45trans(th5_1))
    p41_1 = T14[0:3, 3]
    p41xz_1 = np.hypot(p41_1[0], p41_1[2])

    T14 = np.linalg.inv(T01trans(th1_1)) * des_pos * np.linalg.inv(T56trans(th6_2)) * np.linalg.inv(T45trans(th5_2))
    p41_2 = T14[0:3, 3]
    p41xz_2 = np.hypot(p41_2[0], p41_2[2])

    T14 = np.linalg.inv(T01trans(th1_2)) * des_pos * np.linalg.inv(T56trans(th6_3)) * np.linalg.inv(T45trans(th5_3))
    p41_3 = T14[0:3, 3]
    p41xz_3 = np.hypot(p41_3[0], p41_3[2])

    T14 = np.linalg.inv(T01trans(th1_2)) * des_pos * np.linalg.inv(T56trans(th6_4)) * np.linalg.inv(T45trans(th5_4))
    p41_4 = T14[0:3, 3]
    p41xz_4 = np.hypot(p41_4[0], p41_4[2])

    th3_1 = np.round(
        np.real(cmath.acos((p41xz_1 * p41xz_1 - a[0, 2] * a[0, 2] - a[0, 3] * a[0, 3]) / (2 * a[0, 2] * a[0, 3]))), 4)
    th3_2 = np.round(
        np.real(cmath.acos((p41xz_2 * p41xz_2 - a[0, 2] * a[0, 2] - a[0, 3] * a[0, 3]) / (2 * a[0, 2] * a[0, 3]))), 4)
    th3_3 = np.round(
        np.real(cmath.acos((p41xz_3 * p41xz_3 - a[0, 2] * a[0, 2] - a[0, 3] * a[0, 3]) / (2 * a[0, 2] * a[0, 3]))), 4)
    th3_4 = np.round(
        np.real(cmath.acos((p41xz_4 * p41xz_4 - a[0, 2] * a[0, 2] - a[0, 3] * a[0, 3]) / (2 * a[0, 2] * a[0, 3]))), 4)

    th3_5 = -th3_1
    th3_6 = -th3_2
    th3_7 = -th3_3
    th3_8 = -th3_4

    # *********** theta2 ***********
    th2_1 = -abs(
        np.round(np.real(atan2(-p41_1[2], -p41_1[0])) - np.real(cmath.asin((-a[0, 3] * sin(th3_1)) / p41xz_1)), 4))
    th2_2 = -abs(
        np.round(np.real(atan2(-p41_2[2], -p41_2[0])) - np.real(cmath.asin((-a[0, 3] * sin(th3_2)) / p41xz_2)), 4))
    th2_3 = -abs(
        np.round(np.real(atan2(-p41_3[2], -p41_3[0])) - np.real(cmath.asin((-a[0, 3] * sin(th3_3)) / p41xz_3)), 4))
    th2_4 = -abs(
        np.round(np.real(atan2(-p41_4[2], -p41_4[0])) - np.real(cmath.asin((-a[0, 3] * sin(th3_4)) / p41xz_4)), 4))

    th2_5 = -abs(
        np.round(np.real(atan2(-p41_1[2], -p41_1[0])) - np.real(cmath.asin((a[0, 3] * sin(th3_5)) / p41xz_1)), 4))
    th2_6 = -abs(
        np.round(np.real(atan2(-p41_2[2], -p41_2[0])) - np.real(cmath.asin((a[0, 3] * sin(th3_6)) / p41xz_2)), 4))
    th2_7 = -abs(
        np.round(np.real(atan2(-p41_3[2], -p41_3[0])) - np.real(cmath.asin((a[0, 3] * sin(th3_7)) / p41xz_3)), 4))
    th2_8 = -abs(
        np.round(np.real(atan2(-p41_4[2], -p41_4[0])) - np.real(cmath.asin((a[0, 3] * sin(th3_8)) / p41xz_4)), 4))

    # *********** theta4 ***********
    T34 = np.linalg.inv(T23trans(th3_1)) * np.linalg.inv(T12trans(th2_1)) * np.linalg.inv(T01trans(th1_1)) \
          * des_pos * np.linalg.inv(T56trans(th6_1)) * np.linalg.inv(T45trans(th5_1))
    Xhat43 = T34[0:3, 0]
    th4_1 = np.round(np.real(atan2(Xhat43[1], Xhat43[0])), 4)

    T34 = np.linalg.inv(T23trans(th3_2)) * np.linalg.inv(T12trans(th2_2)) * np.linalg.inv(T01trans(th1_1)) \
          * des_pos * np.linalg.inv(T56trans(th6_2)) * np.linalg.inv(T45trans(th5_2))
    Xhat43 = T34[0:3, 0]
    th4_2 = np.round(np.real(atan2(Xhat43[1], Xhat43[0])), 4)

    T34 = np.linalg.inv(T23trans(th3_3)) * np.linalg.inv(T12trans(th2_3)) * np.linalg.inv(T01trans(th1_2)) \
          * des_pos * np.linalg.inv(T56trans(th6_3)) * np.linalg.inv(T45trans(th5_3))
    Xhat43 = T34[0:3, 0]
    th4_3 = np.round(np.real(atan2(Xhat43[1], Xhat43[0])), 4)

    T34 = np.linalg.inv(T23trans(th3_4)) * np.linalg.inv(T12trans(th2_4)) * np.linalg.inv(T01trans(th1_2)) \
          * des_pos * np.linalg.inv(T56trans(th6_4)) * np.linalg.inv(T45trans(th5_4))
    Xhat43 = T34[0:3, 0]
    th4_4 = np.round(np.real(atan2(Xhat43[1], Xhat43[0])), 4)

    T34 = np.linalg.inv(T23trans(th3_5)) * np.linalg.inv(T12trans(th2_5)) * np.linalg.inv(T01trans(th1_1)) \
          * des_pos * np.linalg.inv(T56trans(th6_1)) * np.linalg.inv(T45trans(th5_1))
    Xhat43 = T34[0:3, 0]
    th4_5 = np.round(np.real(atan2(Xhat43[1], Xhat43[0])), 4)

    T34 = np.linalg.inv(T23trans(th3_6)) * np.linalg.inv(T12trans(th2_6)) * np.linalg.inv(T01trans(th1_1)) \
          * des_pos * np.linalg.inv(T56trans(th6_2)) * np.linalg.inv(T45trans(th5_2))
    Xhat43 = T34[0:3, 0]
    th4_6 = np.round(np.real(atan2(Xhat43[1], Xhat43[0])), 4)

    T34 = np.linalg.inv(T23trans(th3_7)) * np.linalg.inv(T12trans(th2_7)) * np.linalg.inv(T01trans(th1_2)) \
          * des_pos * np.linalg.inv(T56trans(th6_3)) * np.linalg.inv(T45trans(th5_3))
    Xhat43 = T34[0:3, 0]
    th4_7 = np.round(np.real(atan2(Xhat43[1], Xhat43[0])), 4)

    T34 = np.linalg.inv(T23trans(th3_8)) * np.linalg.inv(T12trans(th2_8)) * np.linalg.inv(T01trans(th1_2)) \
          * des_pos * np.linalg.inv(T56trans(th6_4)) * np.linalg.inv(T45trans(th5_4))
    Xhat43 = T34[0:3, 0]
    th4_8 = np.round(np.real(atan2(Xhat43[1], Xhat43[0])), 4)

    th[0, 0],th[1, 0],th[2, 0],th[3, 0],th[4, 0],th[5, 0]=th1_1,th2_1,th3_1,th4_1,th5_1,th6_1
    th[0, 1],th[1, 1],th[2, 1],th[3, 1],th[4, 1],th[5, 1]=th1_1,th2_2,th3_2,th4_2,th5_2,th6_2
    th[0, 2],th[1, 2],th[2, 2],th[3, 2],th[4, 2],th[5, 2]=th1_2,th2_3,th3_3,th4_3,th5_3,th6_3
    th[0, 3],th[1, 3],th[2, 3],th[3, 3],th[4, 3],th[5, 3]=th1_2,th2_4,th3_4,th4_4,th5_4,th6_4
    th[0, 4],th[1, 4],th[2, 4],th[3, 4],th[4, 4],th[5, 4]=th1_1,th2_5,th3_5,th4_5,th5_1,th6_1
    th[0, 5],th[1, 5],th[2, 5],th[3, 5],th[4, 5],th[5, 5]=th1_1,th2_6,th3_6,th4_6,th5_2,th6_2
    th[0, 6],th[1, 6],th[2, 6],th[3, 6],th[4, 6],th[5, 6]=th1_2,th2_7,th3_7,th4_7,th5_3,th6_3
    th[0, 7],th[1, 7],th[2, 7],th[3, 7],th[4, 7],th[5, 7]=th1_2,th2_8,th3_8,th4_8,th5_4,th6_4
    pprint(th)

    # data la configurazione attuale dei joints (passato alla funzione come actual_angles), calcolo l'IK della
    # posizione desiderata e tra le 8 soluzioni, restituisco quella con la "distanza" minore dalla posizione attuale
    # dei joints, ovvero la configurazione che fa muovere di meno gli assi per raggiungere la posizione desiderata
    # controllo anche che la soluzione dell'IK trovata sia entro un certo errore rispetto alla posizione desiderata,
    # poichè al momento, non tutte le soluzioni trovate dall'algoritmo sono giuste (da risolvere)
    dir_kin_angles = [np.array(th[:, 0].flat)]
    actual_norm = np.linalg.norm(actual_angles - dir_kin_angles[0])
    best_angles = dir_kin_angles[0]

    for i in range(8):
        solution = th[:, i]
        dir_kin_angles.append(np.array(solution.flat))
        # calcolo della "distanza" sulla posizione dei joint
        norm = np.linalg.norm(actual_angles - dir_kin_angles[i])
        # calcolo dell'errore di posizionamento tra la posizione voluta e quella che si sta valutando
        ee_pos = direct_kin(dir_kin_angles[i])[0:3, 3].T
        pos_error = np.linalg.norm(ee_pos - des_pos_usd)
        # solo se la "distanza" è buona -E- l'errore è sotto una certa soglia, la soluzione va bene
        if norm <= actual_norm and pos_error < 0.1:
            actual_norm = norm
            best_angles = dir_kin_angles[i]

    return best_angles

def IKTrajectory(jstate, final_p, final_rotm, curve_type='bezier', vel=1):
    initial_pose = direct_kin(jstate)
    initial_p = np.array(initial_pose[0:3, 3].flat)
    initial_rotm = initial_pose[0:3, 0:3]

    ds = 0.05
    if curve_type == 'bezier':
        path, rotms = bezierPath(initial_p, final_p, initial_rotm, final_rotm, int(1/ds))
    elif curve_type == 'line':
        path, rotms = linePath(initial_p, final_p, initial_rotm, final_rotm, int(1/ds))
    elif curve_type == 'parabola':
        path, rotms = parabolaPath(initial_p, final_p, initial_rotm, final_rotm, ds)
    else:
        print('curva non disponibile')
        return

    actual_jstate = np.ndarray.copy(jstate)
    positions = [actual_jstate]

    for i in range(len(path)):
        next_pos = path[i]
        next_rotm = rotms[i]

        next_jstate = inverse_kin(next_pos, next_rotm, actual_jstate)
        positions.append(next_jstate)

        actual_jstate = next_jstate
    
    times = [0.]
    
    for i in range(1, len(positions)):
        dtheta = positions[i] - positions[i-1]
        times.append(times[i-1] + np.amax(abs(dtheta)) / vel)
    
    print(jstate)
    print('-------------------')
    pprint(positions)
    
    return positions, None, times    


# ----------- INVERSE DIFFERENTIAL KINEMATICS ------------------------
def jquintic(self, T, qi, qf, vi, vf, ai, af):
    # traiettoria nello spazio dei joint
    h = qf - qi
    a0 = np.ndarray.copy(qi)
    a1 = np.ndarray.copy(vi)
    a2 = np.ndarray.copy(ai) / 2
    Tinv = np.linalg.inv(
        mat([[T ** 3, T ** 4, T ** 5], [3 * T ** 2, 4 * T ** 3, 5 * T ** 4], [6 * T, 12 * T ** 2, 20 * T ** 3]]))
    a3 = Tinv[0, 0] * (h - vf * T) + Tinv[0, 1] * (vf - vi - ai * T) + Tinv[0, 2] * (af - ai)
    a4 = Tinv[1, 0] * (h - vf * T) + Tinv[1, 1] * (vf - vi - ai * T) + Tinv[1, 2] * (af - ai)
    a5 = Tinv[2, 0] * (h - vf * T) + Tinv[2, 1] * (vf - vi - ai * T) + Tinv[2, 2] * (af - ai)

    times = np.linspace(0, T, 50)

    points = []
    velocities = []
    # calcolo i punti della traiettoria e relative velocità attraverso la quintica
    for t in times:
        points.append(a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5)
        velocities.append(a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3 + 5 * a5 * t ** 4)

    self.des_jstates = points
    self.des_jvels = velocities
    self.times = times

def jcubic(self, T, qi, qf, vi, vf):
    h = qf - qi  # vettore - displacement sui 6 theta tra pos iniziale e finale
    # calcolo i parametri della cubica, per ogni joint
    a0 = np.ndarray.copy(qi)  # vettore [qi1, qi2, qi3, qi4, qi5, qi6]
    a1 = np.ndarray.copy(vi)  # vettore [qf1, qf2, qf3, qf4, qf5, qf6]
    Tinv = np.linalg.inv(mat([[T * T, T * T * T], [2 * T, 3 * T * T]]))
    a2 = Tinv[0, 0] * (h - vi * T) + Tinv[0, 1] * (vf - vi)
    a3 = Tinv[1, 0] * (h - vi * T) + Tinv[1, 1] * (vf - vi)

    times = np.linspace(0, T, 50)

    points = []
    velocities = []
    # calcolo i punti della traiettoria e relative velocità attraverso la cubica
    for t in times:
        points.append(a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3)
        velocities.append(a1 + 2 * a2 * t + 3 * a3 * t ** 2)

    self.des_jstates = points
    self.des_jvels = velocities
    self.times = times

def quinticMovement(self, t, a0, a1, a2, a3, a4, a5):
        return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4 + a5 * t ** 5

def Jacobian(th):
    # Jacobiana dell'ur5 in posizione "a testa in giù", derivata dalla DK customizzata
    # asse z0 e origine frame 0
    z0 = np.array([0, 0, 1])
    p0 = np.array([0, 0, 0])

    # asse z1 e origine frame 1
    T01 = T01mtrans(th[0])
    z1 = np.array(T01[0:3, 2].flat)
    p1 = np.array(T01[0:3, 3].flat)

    # asse z2 e origine frame 2
    T02 = T01 * T12mtrans(th[1])
    z2 = np.array(T02[0:3, 2].flat)
    p2 = np.array(T02[0:3, 3].flat)

    # asse z3 e origine frame 3
    T03 = T02 * T23mtrans(th[2])
    z3 = np.array(T03[0:3, 2].flat)
    p3 = np.array(T03[0:3, 3].flat)

    # asse z4 e origine frame 4
    T04 = T03 * T34mtrans(th[3])
    z4 = np.array(T04[0:3, 2].flat)
    p4 = np.array(T04[0:3, 3].flat)

    # asse z5 e origine frame 5
    T05 = T04 * T45mtrans(th[4])
    z5 = np.array(T05[0:3, 2].flat)
    p5 = np.array(T05[0:3, 3].flat)

    # asse z6 e origine frame 6
    T06 = T05 * T56mtrans(th[5])
    z6 = np.array(T06[0:3, 2].flat)
    p6 = np.array(T06[0:3, 3].flat)

    J = np.zeros((6, 6))
    z1xp = np.cross(z1, p6 - p1)
    J[0:3, 0] = z1xp.T
    J[3:6, 0] = z1.T

    z2xp = np.cross(z2, p6 - p2)
    J[0:3, 1] = z2xp.T
    J[3:6, 1] = z2.T

    z3xp = np.cross(z3, p6 - p3)
    J[0:3, 2] = z3xp.T
    J[3:6, 2] = z3.T

    z4xp = np.cross(z4, p6 - p4)
    J[0:3, 3] = z4xp.T
    J[3:6, 3] = z4.T

    z5xp = np.cross(z5, p6 - p5)
    J[0:3, 4] = z5xp.T
    J[3:6, 4] = z5.T

    z6xp = np.cross(z6, p6 - p6)
    J[0:3, 5] = z6xp.T
    J[3:6, 5] = z6.T

    return J

def cubicCoeff(T, start_pos, end_pos, start_vel=np.array([0, 0, 0]), end_vel=np.array([0, 0, 0])):
    h = end_pos - start_pos  # vettore - displacement su x,y,z
    # calcolo i parametri della cubica, per x,y,z
    a0 = np.ndarray.copy(start_pos)  # vettore [a0x,a0y,a0z]
    a1 = np.ndarray.copy(start_vel)
    Tinv = np.linalg.inv(mat([[T * T, T * T * T], [2 * T, 3 * T * T]]))
    a2 = Tinv[0, 0] * (h - start_vel * T) + Tinv[0, 1] * (end_vel - start_vel)
    a3 = Tinv[1, 0] * (h - start_vel * T) + Tinv[1, 1] * (end_vel - start_vel)

    return a0, a1, a2, a3

def quinticCoeff(T, start_pos, end_pos, start_vel=np.array([0, 0, 0]), end_vel=np.array([0, 0, 0]),
                 start_acc=np.array([0, 0, 0]), end_acc=np.array([0, 0, 0])):
    h = end_pos - start_pos
    a0 = np.ndarray.copy(start_pos)
    a1 = np.ndarray.copy(start_vel)
    a2 = np.ndarray.copy(start_acc) / 2

    Tinv = np.linalg.inv(
        mat([[T ** 3, T ** 4, T ** 5], [3 * T ** 2, 4 * T ** 3, 5 * T ** 4], [6 * T, 12 * T ** 2, 20 * T ** 3]]))

    a3 = Tinv[0, 0] * (h - end_vel * T) + Tinv[0, 1] * (end_vel - start_vel - start_acc * T) + Tinv[0, 2] * (
            end_acc - start_acc)
    a4 = Tinv[1, 0] * (h - end_vel * T) + Tinv[1, 1] * (end_vel - start_vel - start_acc * T) + Tinv[1, 2] * (
            end_acc - start_acc)
    a5 = Tinv[2, 0] * (h - end_vel * T) + Tinv[2, 1] * (end_vel - start_vel - start_acc * T) + Tinv[2, 2] * (
            end_acc - start_acc)

    return a0, a1, a2, a3, a4, a5

def velocityCubic(t, a1, a2, a3):
    return a1 + 2 * a2 * t + 3 * a3 * t ** 2

def velocityQuintic(t, a1, a2, a3, a4, a5):
    return a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3 + 5 * a5 * t ** 4

def eul2rotm(euler_angles):
    # euler angles definiti come [rot_x, rot_y, rot_z]

    rotx = mat([[1, 0, 0],
                [0, cos(euler_angles[0]), -sin(euler_angles[0])],
                [0, sin(euler_angles[0]), cos(euler_angles[0])]])

    roty = mat([[cos(euler_angles[1]), 0, sin(euler_angles[1])],
                [0, 1, 0],
                [-sin(euler_angles[1]), 0, cos(euler_angles[1])]])

    rotz = mat([[cos(euler_angles[2]), -sin(euler_angles[2]), 0],
                [sin(euler_angles[2]), cos(euler_angles[2]), 0],
                [0, 0, 1]])

    return rotz * roty * rotx

def rpy2rotm(rpy_angles):
    # rpy angles definiti come [roll, pitch, yaw]

    rotx = mat([[1, 0, 0],
                [0, cos(rpy_angles[0]), -sin(rpy_angles[0])],
                [0, sin(rpy_angles[0]), cos(rpy_angles[0])]])

    roty = mat([[cos(rpy_angles[1]), 0, sin(rpy_angles[1])],
                [0, 1, 0],
                [-sin(rpy_angles[1]), 0, cos(rpy_angles[1])]])

    rotz = mat([[cos(rpy_angles[2]), -sin(rpy_angles[2]), 0],
                [sin(rpy_angles[2]), cos(rpy_angles[2]), 0],
                [0, 0, 1]])

    return rotx * roty * rotz

def rotm2eul(R):
    if R[2, 0] != 1 and R[2, 0] != -1:
        theta1 = -np.real(cmath.asin(R[2, 0]))
        theta2 = pi - theta1

        csi1 = atan2(R[2, 1] / cos(theta1), R[2, 2] / cos(theta1))
        csi2 = atan2(R[2, 1] / cos(theta2), R[2, 2] / cos(theta2))

        phi1 = atan2(R[1, 0] / cos(theta1), R[0, 0] / cos(theta1))
        phi2 = atan2(R[1, 0] / cos(theta2), R[0, 0] / cos(theta2))
    else:
        phi1 = 0
        if R[2, 0] == -1:
            theta1 = pi / 2
            csi1 = phi1 + atan2(R[0, 1], R[0, 2])
        else:
            theta1 = -pi / 2
            csi1 = -phi1 + atan2(-R[0, 1], -R[0, 2])

    return np.array([csi1, theta1, phi1])

def quat2rotm(q):
    return mat([[q[0] ** 2 + q[1] ** 2 - q[2] ** 2 - q[3] ** 2, 2 * (q[1] * q[2] - q[0] * q[3]),
                 2 * (q[1] * q[3] + q[0] * q[2])],
                [2 * (q[1] * q[2] + q[0] * q[3]), q[0] ** 2 - q[1] ** 2 + q[2] ** 2 - q[3] ** 2,
                 2 * (q[2] * q[3] - q[0] * q[1])],
                [2 * (q[1] * q[3] - q[0] * q[2]), 2 * (q[2] * q[3] + q[0] * q[1]),
                 q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2]])

def quat2rpy(x, y, z, w):
    roll = np.round(math.atan2(2*(w*x+y*z), w**2 - x**2 - y**2 + z**2), 4)
    pitch = np.round(np.real(cmath.asin(2*(w*y-x*z))), 4)
    if pi/2-0.001 < pitch < pi/2+0.001 or -pi/2-0.001 < pitch < -pi/2+0.001:
        roll = 0
        yaw = np.round(-2*np.sign(pitch)*math.atan2(x, w), 4)
    else:
        yaw = np.round(math.atan2(2*(w*z+x*y), w**2 + x**2 - y**2 - z**2), 4)
    
    return np.array([roll, pitch, yaw])

def controlledQdot(jstate, actual_pos, des_pos, des_vel, Kp):
    J = Jacobian(jstate)
    J = J[0:3, 0:6]
    piJ = np.linalg.pinv(J)
    # print('pdes   - apos: ', des_pos, actual_pos)

    return np.dot(piJ, (des_vel + np.dot(Kp * np.eye(3), (des_pos - actual_pos))))

def dw_dq(jstate):
    qM = np.array([6.14, 0, pi])
    qm = np.array([-6.14, -pi, -pi])
    qbar = (qM + qm) / 2

    dwdq = np.zeros(6)

    for i in range(3):
        dwdq[i] = (-1 / 3.) * (jstate[i] - qbar[i]) / ((qM[i] - qm[i]) ** 2)

    return dwdq

def controlledQdotRedundant(jstate, actual_pos, des_pos, des_vel, K0):
    J = Jacobian(jstate)
    J = J[0:3, 0:6]
    piJ = np.linalg.pinv(J)

    qdot0 = K0 * dw_dq(jstate)

    return np.dot(piJ, (des_vel + np.dot(K0 * np.eye(3), (des_pos - actual_pos)))) + np.dot(np.eye(6) - np.dot(piJ, J),
                                                                                            qdot0)

def controlledQdotComplete(jstate, actual_pos, des_pos, des_vel, actual_phi, des_phi, des_phidot, Kp, Kphi):
    J = Jacobian(jstate)

    phi = actual_phi[2]
    theta = actual_phi[1]

    T = mat([[cos(theta) * cos(phi), -sin(phi), 0],
             [cos(theta) * sin(phi), cos(phi), 0],
             [-sin(theta), 0, 1]])

    Ta = np.zeros((6, 6))
    Ta[0:3, 0:3] = np.eye(3)
    Ta[3:6, 3:6] = T
    TaT = np.linalg.inv(Ta)

    Ja = np.dot(TaT, J)

    # svd = np.linalg.svd(Ja, compute_uv=False)
    # min_sing_val = np.amin(svd)
    # max_sing_val = np.amax(svd)
    # # print('msv: ', min_sing_val)
    # #damping_factor = max_sing_val / min_sing_val
    #
    # epsilon = 0.15
    # lambda_max = 0.001
    # if min_sing_val < epsilon:  # 0.000387 valore a cui si spacca qualcosa
    #     # DA MIGLIORARE E CONTROLLARE: ottimizzazione per stare lontani da singolarità (e se dentro una, uscirne)
    #     # print('correzione')
    #
    #     lambda_G = np.zeros(6)
    #     for i in range(6):
    #         exp = math.e**(-(svd[i]/epsilon)**2)
    #         lambda_G[i] = (lambda_max * exp)**2
    #
    #     # print('lambda: ', lambda_G[-1])
    #
    #     JaT = np.transpose(Ja)
    #     JaJaT = np.matmul(Ja, JaT)
    #     #pprint(JaJaT)
    #
    #     correction = lambda_G * np.eye(6)
    #     corrected_Ja = JaJaT + correction
    #     JaStar = np.matmul(JaT, np.linalg.inv(corrected_Ja))
    #     #pprint(JaStar)
    #     Ja = JaStar

    # todo: implementare la modulazione del coefficiente in base alla distanza
    dotxe = np.zeros(6)
    # print('pdes   - apos: ', des_pos, actual_pos)
    # print('phides - aphi: ', des_phi, actual_phi)

    dotxe[0:3] = des_vel + np.dot(Kp * np.eye(3), (des_pos - actual_pos))
    dotxe[3:6] = des_phidot + np.dot(Kphi * np.eye(3), (des_phi - actual_phi))

    qdot = np.dot(np.linalg.inv(Ja), dotxe)
    # print('qdot: ', qdot)

    return qdot

def lenghtOfParabola(a, b, c, xi, xf):
    nf = math.sqrt((2 * a * xf + b) ** 2 + 1) * (2 * a * xf + b) + np.arcsinh(2 * a * xf + b)
    lf = nf / (4 * a)
    ni = math.sqrt((2 * a * xi + b) ** 2 + 1) * (2 * a * xi + b) + np.arcsinh(2 * a * xi + b)
    li = ni / (4 * a)
    return lf - li

def sistemOfEquations(vars, *data):
    l, xi, yi, a, b, c = data
    xf, yf = vars
    eq1 = (xf - xi) ** 2 + (yf - yi) ** 2 - l ** 2
    eq2 = a * xf ** 2 + b * xf + c - yf
    return [eq1, eq2]

def parabolaPath(pi, pf, rotm_i, rotm_f, ds):
    # dati due punti pi e pf (punto iniziale e punto finale nello spazio xyz) riferiti al base frame
    # questa funzione calcola n punti appartenenti ad un path parabolico, ognuno distante circa ds l'uno dall'altro
    # input: pi, pf punti nello spazio xyz riferiti al base frame
    # output: una lista di punti nello spazio xyz riferiti al base frame appartenenti ad un path parabolico
    pi = np.array([pi[0], pi[1], pi[2], 1])
    pf = np.array([pf[0], pf[1], pf[2], 1])

    pv = (pi + pf) / 2
    pv[1] = pv[1] * 1.5

    base_p = pf - pi
    xp = base_p / np.linalg.norm(base_p)

    pipv = (pv - pi) / np.linalg.norm(pv - pi)
    zp = np.cross(xp[0:3], pipv[0:3])
    zp = np.array([zp[0], zp[1], zp[2], 1])

    theta = atan2(zp[0], zp[2])

    q = np.array([cos(-theta / 2), 0, sin(-theta / 2), 0])
    rotmY = quat2rotm(q)

    xp_xy = np.array(np.dot(rotmY, xp[0:3]).flat)

    phi = atan2(xp_xy[1], xp_xy[0])

    rotY = mat([[cos(theta), 0, sin(theta), 0],
                [0, 1, 0, 0],
                [-sin(theta), 0, cos(theta), 0],
                [0, 0, 0, 1]])
    rotZ = mat([[cos(phi), -sin(phi), 0, 0],
                [sin(phi), cos(phi), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])
    traslO = mat([[0, 0, 0, pi[0]],
                  [0, 0, 0, pi[1]],
                  [0, 0, 0, pi[2]],
                  [0, 0, 0, 0]])

    Tbc = np.matmul(rotY, rotZ) + traslO
    Tcb = np.linalg.inv(Tbc)

    pic = np.array(np.dot(Tcb, pi).flat)
    pvc = np.array(np.dot(Tcb, pv).flat)
    pfc = np.array(np.dot(Tcb, pf).flat)

    coeff = mat([[pic[0] ** 2, pic[0], 1],
                 [pvc[0] ** 2, pvc[0], 1],
                 [pfc[0] ** 2, pfc[0], 1]])

    [a, b, c] = np.array(np.dot(np.linalg.inv(coeff), np.array([pic[1], pvc[1], pfc[1]])).flat)

    l = lenghtOfParabola(a, b, c, pic[0], pfc[0])
    L = np.linspace(0, l, int(l / ds))
    lhat = L[1]

    xi, yi = 0, 0
    path = [[xi, yi, 0, 1]]  # ridondante, possibilmente da togliere e sostituire con una variabile temporanea
    pathBF = [np.array(np.dot(Tbc, path[0]).flat)]
    i = 1
    while yi >= 0:
        xfhat = xi + lhat / 2
        yfhat = a * xfhat ** 2 + b * xfhat + c
        xf, yf = fsolve(sistemOfEquations, [xfhat, yfhat], args=(lhat, xi, yi, a, b, c))
        xi = xf
        yi = a * xi ** 2 + b * xi + c
        path.append([xi, yi, 0, 1])
        pathBF.append(np.array(np.dot(Tbc, path[i]).flat))
        i = i + 1

    path = path[0:len(path) - 1]
    pathBF = pathBF[0:len(pathBF) - 1]

    path.append([pfc[0], pfc[1], 0, 1])
    pathBF.append(np.array(np.dot(Tbc, path[-1]).flat))

    rotms = matrixLinspace(rotm_i, rotm_f, int(1/ds))

    return pathBF, rotms

def matrixLinspace(rotm_i, rotm_f, n):
    x_dir = np.linspace(np.array(rotm_i[0:3, 0].flat), np.array(rotm_f[0:3, 0].flat), n)
    y_dir = np.linspace(np.array(rotm_i[0:3, 1].flat), np.array(rotm_f[0:3, 1].flat), n)
    z_dir = np.linspace(np.array(rotm_i[0:3, 2].flat), np.array(rotm_f[0:3, 2].flat), n)

    mlinspace = []
    for i in range(n):
        m = np.eye(3)
        m[0:3, 0] = x_dir[i]
        m[0:3, 1] = y_dir[i]
        m[0:3, 2] = z_dir[i]
        mlinspace.append(m)

    return mlinspace

def binomialCoeff(n, k):
    return math.factorial(n)/(math.factorial(k)*math.factorial(n-k))

def bezierEquation(n, t, p):
    b = np.zeros(3)
    for k in range(n+1):
        bin = binomialCoeff(n, k)
        b += bin * ((1-t)**(n-k))*(t**k)*p[k]

    return b

def bezierPath(start_pos, end_pos, rotm_i, rotm_f, n):
    T = np.linspace(0, 1, n)
    points = []
    alpha = 0
    done=False

    while (not done):
        done=True
        mid_pos = np.array([start_pos[0]+(end_pos[0]-start_pos[0])/10,
                        max(start_pos[1],end_pos[1]) -1.19*max(start_pos[1],end_pos[1])+1 +alpha,
                            start_pos[2]+(end_pos[2]-start_pos[2])/2])
        
        mid4_pos = np.array([start_pos[0]+(end_pos[0]-start_pos[0])*9/10,
                        max(start_pos[1],end_pos[1]) -1.19*max(start_pos[1],end_pos[1])+1 +alpha,
                            start_pos[2]+(end_pos[2]-start_pos[2])/2])

        mid2_pos = np.array([end_pos[0], end_pos[1]-0.1,
                            end_pos[2]+0.3])

        mid3_pos = np.array([start_pos[0], start_pos[1],
                            start_pos[2]+0.25])

        p = np.array([start_pos, mid3_pos, mid_pos, mid4_pos, mid2_pos, end_pos])

        for t in T:
            b = bezierEquation(5, t, p)
            rb=math.sqrt(b[0]**2+b[1]**2)
            if (rb<0.10):
                alpha+0.05
                points.clear()
                print("new correction")
                done=False
                break

            points.append(b)
            
    rotms = matrixLinspace(rotm_i, rotm_f, n)

    return points, rotms

def linePath(start_pos, end_pos, rotm_i, rotm_f, n):
    return np.linspace(start_pos, end_pos, n), matrixLinspace(rotm_i, rotm_f, n)

def skewOperator(w):
    return mat([[    0, -w[2],  w[1]],
                [ w[2],     0, -w[0]],
                [-w[1],  w[0],    0]])

def veeOperator(R):
    return np.array([R[2, 1], R[0, 2], R[1, 0]])

def getDirectionStep(actual_pose, desired_pose):
    T = desired_pose @ np.linalg.inv(actual_pose)
    R = T[0:3, 0:3]

    theta = np.real(cmath.acos((np.trace(R)-1) / 2))

    if np.round(theta, 5) == 0:
        w = [0, 0, 0]
    else:
        w = np.dot(theta / (2*sin(theta)), veeOperator(R - np.transpose(R)))

    v = np.array(desired_pose[0:3, 3].flat) - np.array(actual_pose[0:3, 3].flat)

    return np.array([v[0], v[1], v[2], w[0], w[1], w[2]])

def invDiffKin(jstate, desired_pose, precision, damping=0.04, max_delta=0.032, time_scale=1):
    next_jstate = jstate
    actual_pose = direct_kin(jstate)

    delta = np.linalg.norm(actual_pose - desired_pose)

    positions = [jstate]

    while delta > precision:
        jstate = next_jstate

        step = getDirectionStep(actual_pose, desired_pose)

        J = Jacobian(jstate)

        svd = np.linalg.svd(J, compute_uv=False)
        min_sing_val = np.amin(svd)

        if min_sing_val < 0.001:
            dtheta = np.transpose(J) @ np.linalg.inv(J @ np.transpose(J) + np.dot(damping**2, np.eye(6))) @ step
            print('correzione singolarità')
        else:
            dtheta = np.linalg.inv(J) @ step

        scale_delta = max_delta / max(max_delta, np.amax(abs(dtheta)))
        scaled_dtheta = np.dot(time_scale*scale_delta, dtheta)

        next_jstate = jstate + scaled_dtheta

        positions.append(next_jstate)

        previous_pose = np.ndarray.copy(actual_pose)
        actual_pose = direct_kin(next_jstate)

        delta = np.linalg.norm(actual_pose - previous_pose)

    positions = positions[0:-1]

    return positions

def differential_kin(initial_jstate, final_p, final_rotm, curve_type='bezier', vel=1):
    initial_pose = direct_kin(initial_jstate)
    initial_p = np.array(initial_pose[0:3, 3].flat)
    initial_rotm = initial_pose[0:3, 0:3]

    ds = 0.05
    if curve_type == 'bezier':
        path, rotms = bezierPath(initial_p, final_p, initial_rotm, final_rotm, int(1/ds))
    elif curve_type == 'line':
        path, rotms = linePath(initial_p, final_p, initial_rotm, final_rotm, int(1/ds))
    elif curve_type == 'parabola':
        path, rotms = parabolaPath(initial_p, final_p, initial_rotm, final_rotm, ds)
    else:
        print('curva non disponibile')
        return

    actual_jstate = np.ndarray.copy(initial_jstate)
    positions = [actual_jstate]

    intermediate_precision = 0.005
    final_precision = 0.001

    for i in range(1, len(path)):
        next_pos = path[i][0:3]
        next_rotm = rotms[i]

        next_pose = np.eye(4)
        next_pose[0:3, 0:3] = next_rotm
        next_pose[0:3, 3] = next_pos

        if i < len(path) - 3:
            new_viapoints = invDiffKin(actual_jstate, next_pose, intermediate_precision)
        else:
            new_viapoints = invDiffKin(actual_jstate, next_pose, final_precision)

        if len(new_viapoints) <= 1:
            continue
        else:
            diff = positions[-1] - new_viapoints[0]
            if np.linalg.norm(diff) < 0.0005:
                new_viapoints = new_viapoints[1:]

        positions = positions + new_viapoints
        actual_jstate = new_viapoints[-1]

        #des_jvel = controlledQdot(actual_jstate, actual_pos, des_pos, des_vel_p, Kp)
        # print('des jvel: ', des_jvel)
        # des_jvel = controlledQdotRedundant(actual_jstate, actual_pos, des_pos, des_vel_p, 50)
        # des_jvel = controlledQdotphi(actual_jstate, actual_phi, des_phi, des_vel_phi, Kphi)
        # des_jvel = controlledQdotComplete(actual_jstate, actual_pos, des_pos, des_vel_p, actual_phi, des_phi, des_vel_phi, Kp, Kphi)
        # velocities.append(des_jvel)

        # passo di integrazione: calcolo il prossimo jstate usando l'attuale e sommando lo step con le adeguate velocità
        # next_jstate = actual_jstate + des_jvel * (t-times[i-1])
        # next_jstate = actual_jstate + des_jvel * t

        # metto come attuale jstate quello che ho appena calcolato e passo al prossimo punto
        # actual_jstate = next_jstate

    # times = np.round(np.linspace(0, np.round(len(positions)/10, 5), len(positions)), 5)
    
    times = [0.]
    velocities = [np.array([vel, vel, vel, vel, vel, vel])]
    for i in range(1, len(positions)):
        dtheta = positions[i] - positions[i-1]
        times.append(times[i-1] + np.amax(abs(dtheta)) / vel)
        #print(dtheta, ' - ', times[i])
        velocities.append(np.array([vel, vel, vel, vel, vel, vel]))
    
    return positions, velocities, times


