"""
linear.py
Trayectoria lineal en espacio cartesiano.

MATLAB equivalente: LIN.m

Estrategia
----------
Interpola linealmente la posición del efector final (px, py, pz) entre
el punto inicial y el punto final en n pasos iguales. En cada paso
resuelve la cinemática inversa para obtener los ángulos de junta y
llama a la cinemática directa para obtener el estado completo del robot.

Las juntas de orientación (q4, q5, q6) se mantienen constantes durante
el movimiento, igual que en el MATLAB original.
"""

import numpy as np
from typing import Generator

from kinematics.forward import forward_kinematics, ForwardKinematicsResult
from kinematics.inverse import inverse_kinematics, SingularityError


def linear_trajectory(
    p_start: list,
    p_end: list,
    q_current_deg: list,
    duration: float = 2.0,
    dt: float = 0.1,
) -> Generator[ForwardKinematicsResult, None, None]:
    """
    Genera la trayectoria lineal cartesiana como secuencia de estados FK.

    Parámetros
    ----------
    p_start : list[float]
        Posición inicial [px, py, pz] del efector final.
    p_end : list[float]
        Posición final [px, py, pz] del efector final.
    q_current_deg : list[float]
        Ángulos actuales [q1..q6] en GRADOS. Se usan q4, q5, q6 como
        orientación constante y para obtener A1/A2/A3 iniciales.
    duration : float
        Duración total de la trayectoria en segundos. Equivale al
        parámetro T del MATLAB original.
    dt : float
        Paso de tiempo en segundos (0.1 en el MATLAB original).

    Yields
    ------
    ForwardKinematicsResult
        Estado completo del robot en cada instante.

    Lanza
    -----
    SingularityError
        Si algún punto intermedio está fuera del espacio de trabajo.
    """
    po = np.array(p_start, dtype=float)
    pf = np.array(p_end,   dtype=float)

    # Orientación constante durante toda la trayectoria (igual que LIN.m)
    q4, q5, q6 = q_current_deg[3], q_current_deg[4], q_current_deg[5]

    # Estado inicial FK para obtener orientación (alpha, beta, gamma) y matrices A
    fk_init = forward_kinematics(*q_current_deg)
    alpha = fk_init.orientation["alpha"]
    beta  = fk_init.orientation["beta"]
    gamma = fk_init.orientation["gamma"]

    # Número de pasos — igual a T/dt del MATLAB
    n = max(int(duration / dt), 1)
    delta = (pf - po) / n

    p = po.copy()
    for i in range(n + 1):
        # Cinemática inversa en la posición cartesiana actual
        fk_prev = forward_kinematics(*q_current_deg)
        ik = inverse_kinematics(
            p[0], p[1], p[2],
            alpha, beta, gamma,
            fk_prev.A1, fk_prev.A2, fk_prev.A3,
        )

        # Cinemática directa con los nuevos ángulos del brazo + orientación fija
        fk = forward_kinematics(
            ik.q1_deg, ik.q2_deg, ik.q3_deg,
            q4, q5, q6,
        )
        q_current_deg = fk.joints_deg

        yield fk

        # Garantizar que el último punto sea exactamente pf
        if i < n:
            p = po + delta * (i + 1)
        else:
            p = pf.copy()