"""
forward.py
Cinemática directa del robot de 6 DOF.

MATLAB equivalente: fcdirecta.m
Nota: este módulo contiene SOLO la matemática. El graficado y el
control de Arduino son responsabilidad del frontend y de hardware/arduino.py.

Configuración del robot
-----------------------
L1 = 5  (altura de la base, desplazamiento d del eslabón 1)
L2 = 5  (longitud del eslabón 2)
L3 = 5  (longitud del eslabón 3)
"""

import numpy as np
from dataclasses import dataclass

from kinematics.dh import dh_matrix

# Parámetros geométricos del robot (constantes hardcodeadas en el MATLAB original)
L1: float = 5.0
L2: float = 5.0
L3: float = 5.0


@dataclass
class ForwardKinematicsResult:
    """
    Resultado de la cinemática directa.

    Atributos
    ---------
    joints_deg : list[float]
        Ángulos de las 6 juntas en grados (redondeados).
    position : dict
        Posición del efector final {"px", "py", "pz"} redondeada.
    orientation : dict
        Orientación del EF en ángulos ZYX {"alpha", "beta", "gamma"} en radianes.
    links : list[dict]
        Segmentos del robot para graficar. Cada dict tiene "from" y "to"
        como listas [x, y, z]. Útil para Three.js.
    transform : list[list[float]]
        Matriz de transformación homogénea T (4x4) completa.
    A1 : np.ndarray
        Matriz de transformación del eslabón 1 (necesaria para cinemática inversa).
    A2 : np.ndarray
        Matriz de transformación acumulada hasta el eslabón 2.
    A3 : np.ndarray
        Matriz de transformación acumulada hasta el eslabón 3.
    """
    joints_deg: list
    position: dict
    orientation: dict
    links: list
    transform: list
    A1: np.ndarray
    A2: np.ndarray
    A3: np.ndarray


def forward_kinematics(
    q1_deg: float,
    q2_deg: float,
    q3_deg: float,
    q4_deg: float,
    q5_deg: float,
    q6_deg: float,
) -> ForwardKinematicsResult:
    """
    Calcula la cinemática directa del robot.

    Parámetros
    ----------
    q1_deg .. q6_deg : float
        Ángulos de las 6 juntas en GRADOS.

    Retorna
    -------
    ForwardKinematicsResult
        Posición, orientación, segmentos para graficar y matrices DH.
    """
    q1 = np.deg2rad(q1_deg)
    q2 = np.deg2rad(q2_deg)
    q3 = np.deg2rad(q3_deg)
    q4 = np.deg2rad(q4_deg)
    q5 = np.deg2rad(q5_deg)
    q6 = np.deg2rad(q6_deg)

    # Matrices DH individuales (mismos parámetros que fcdirecta.m)
    A1 = dh_matrix(q1, L1, 0,  np.pi / 2)
    A2 = dh_matrix(q2, 0,  L2, 0)
    A3 = dh_matrix(q3, 0,  L3, 0)
    A4 = dh_matrix(0,  0,  0,  q4)
    A5 = dh_matrix(q5, 0,  0, -np.pi / 2)
    A6 = dh_matrix(q6, 0,  0,  0)

    # Transformaciones acumuladas
    A21 = A1 @ A2
    A321 = A21 @ A3
    T = A321 @ A4 @ A5 @ A6

    # Posición de cada junta (para graficar los eslabones)
    x1, y1, z1 = A1[0, 3], A1[1, 3], A1[2, 3]
    x2, y2, z2 = A21[0, 3], A21[1, 3], A21[2, 3]
    px, py, pz = T[0, 3], T[1, 3], T[2, 3]

    # Orientación del efector final (ángulos de Euler ZYX)
    alpha = np.arctan2(T[2, 1], T[1, 1])   # roll
    beta  = np.arctan2(-T[0, 1], np.sqrt(T[1, 1]**2 + T[2, 1]**2))  # pitch
    gamma = np.arctan2(T[0, 2], T[0, 0])   # yaw

    return ForwardKinematicsResult(
        joints_deg=[
            round(np.rad2deg(q1), 2),
            round(np.rad2deg(q2), 2),
            round(np.rad2deg(q3), 2),
            round(np.rad2deg(q4), 2),
            round(np.rad2deg(q5), 2),
            round(np.rad2deg(q6), 2),
        ],
        position={"px": round(px, 2), "py": round(py, 2), "pz": round(pz, 2)},
        orientation={"alpha": alpha, "beta": beta, "gamma": gamma},
        links=[
            {"from": [0.0,  0.0,  0.0], "to": [x1, y1, z1]},  # base → junta 1
            {"from": [x1,   y1,   z1],  "to": [x2, y2, z2]},  # junta 1 → 2
            {"from": [x2,   y2,   z2],  "to": [px, py, pz]},  # junta 2 → EF
        ],
        transform=T.tolist(),
        A1=A1,
        A2=A2,
        A3=A3,
    )