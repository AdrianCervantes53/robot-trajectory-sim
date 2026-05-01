"""
circular.py
Trayectoria circular en espacio cartesiano (3 puntos definen el arco).

MATLAB equivalente: CIR2D.m

Estrategia
----------
Dados 3 puntos en el espacio (inicio, intermedio, fin), se calcula el
círculo que los contiene usando la fórmula directa del circuncentro —
sin álgebra simbólica (el MATLAB original usaba `solve` con sympy).
El arco se recorre en dos tramos: po→pa y pa→pf, respetando la
dirección y sentido correctos para pasar por el punto intermedio.

El movimiento es siempre en un plano 2D proyectado según el parámetro
`plane`: XY (1), XZ (2), YZ (3).

Planos soportados
-----------------
  plane=1  →  XY  (varía px, py; pz constante)
  plane=2  →  XZ  (varía px, pz; py constante)
  plane=3  →  YZ  (varía py, pz; px constante)
"""

import numpy as np
from typing import Generator

from kinematics.forward import forward_kinematics, ForwardKinematicsResult
from kinematics.inverse import inverse_kinematics, SingularityError


# ─────────────────────────────────────────────────────────────
# Geometría del círculo
# ─────────────────────────────────────────────────────────────

def _circumcenter(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray):
    """
    Calcula el centro y radio del círculo que pasa por 3 puntos 2D.

    Reemplaza el `solve(E1, E2, E3)` simbólico de MATLAB con la fórmula
    directa del circuncentro. Más rápido y sin dependencias simbólicas.

    Parámetros
    ----------
    p1, p2, p3 : np.ndarray
        Puntos 2D [x, y].

    Retorna
    -------
    (center, radius) : (np.ndarray shape (2,), float)

    Lanza
    -----
    ValueError si los 3 puntos son colineales (radio infinito).
    """
    ax, ay = p1
    bx, by = p2
    cx, cy = p3

    D = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(D) < 1e-10:
        raise ValueError("Los 3 puntos son colineales — no definen un círculo único.")

    ux = ((ax**2 + ay**2) * (by - cy) +
          (bx**2 + by**2) * (cy - ay) +
          (cx**2 + cy**2) * (ay - by)) / D

    uy = ((ax**2 + ay**2) * (cx - bx) +
          (bx**2 + by**2) * (ax - cx) +
          (cx**2 + cy**2) * (bx - ax)) / D

    center = np.array([ux, uy])
    radius = np.linalg.norm(p1 - center)
    return center, radius


def _arc_direction(theta1: float, theta2: float, theta3: float):
    """
    Determina la dirección (±1) y los ángulos ajustados para recorrer
    el arco theta1 → theta2 → theta3 pasando por el punto intermedio.

    Replica la lógica de los 6 casos if/elseif de CIR2D.m.

    Retorna
    -------
    (k, theta2_adj, theta3_adj) donde k=+1 antihorario, k=-1 horario.
    """
    t1, t2, t3 = theta1, theta2, theta3

    if t3 > t2 > t1:          # caso 1 — antihorario directo
        return +1, t2, t3
    elif t1 > t2 > t3:        # caso 2 — horario directo
        return -1, t2, t3
    elif t1 > t3 > t2:        # caso 3
        return +1, t2 + 360, t3
    elif t2 > t3 > t1:        # caso 4
        return -1, t2 - 360, t3
    elif t2 > t1 > t3:        # caso 5
        return +1, t2, t3 + 360
    else:                      # caso 6  (t3 > t1 > t2)
        return -1, t2, t3 - 360


# ─────────────────────────────────────────────────────────────
# Proyección de plano
# ─────────────────────────────────────────────────────────────

def _project(p_3d: list, plane: int):
    """Extrae las 2 coordenadas activas según el plano seleccionado."""
    px, py, pz = p_3d
    if plane == 1:
        return np.array([px, py])
    elif plane == 2:
        return np.array([px, pz])
    else:
        return np.array([py, pz])


def _unproject(xy: np.ndarray, p_ref: list, plane: int) -> tuple:
    """
    Reconstruye (px, py, pz) a partir de las 2 coordenadas del arco
    y la coordenada constante tomada de la posición de referencia.
    """
    px_ref, py_ref, pz_ref = p_ref
    if plane == 1:
        return xy[0], xy[1], pz_ref
    elif plane == 2:
        return xy[0], py_ref, xy[1]
    else:
        return px_ref, xy[0], xy[1]


# ─────────────────────────────────────────────────────────────
# Trayectoria circular
# ─────────────────────────────────────────────────────────────

def circular_trajectory(
    p_start: list,
    p_mid: list,
    p_end: list,
    q_current_deg: list,
    plane: int = 1,
    step_deg: float = 1.0,
) -> Generator[ForwardKinematicsResult, None, None]:
    """
    Genera la trayectoria circular como secuencia de estados FK.

    Parámetros
    ----------
    p_start : list[float]
        Posición inicial [px, py, pz] del efector final.
    p_mid : list[float]
        Punto intermedio [px, py, pz] por donde pasa el arco.
    p_end : list[float]
        Posición final [px, py, pz] del efector final.
    q_current_deg : list[float]
        Ángulos actuales [q1..q6] en GRADOS.
    plane : int
        Plano de movimiento: 1=XY, 2=XZ, 3=YZ.
    step_deg : float
        Incremento angular en grados por paso (1° en el MATLAB original).

    Yields
    ------
    ForwardKinematicsResult
        Estado completo del robot en cada instante del arco.

    Lanza
    -----
    ValueError
        Si los 3 puntos son colineales.
    SingularityError
        Si algún punto del arco está fuera del espacio de trabajo.
    """
    # Proyectar los 3 puntos al plano seleccionado
    pt1 = _project(p_start, plane)
    pt2 = _project(p_mid,   plane)
    pt3 = _project(p_end,   plane)

    # Centro y radio del círculo
    center, radius = _circumcenter(pt1, pt2, pt3)
    xc, yc = center

    # Ángulos de cada punto respecto al centro (en grados)
    theta1 = np.rad2deg(np.arctan2(pt1[1] - yc, pt1[0] - xc))
    theta2 = np.rad2deg(np.arctan2(pt2[1] - yc, pt2[0] - xc))
    theta3 = np.rad2deg(np.arctan2(pt3[1] - yc, pt3[0] - xc))

    # Dirección y ángulos ajustados para seguir el arco correctamente
    k, theta2_adj, theta3_adj = _arc_direction(theta1, theta2, theta3)

    # Orientación constante (q4, q5, q6) durante toda la trayectoria
    q4, q5, q6 = q_current_deg[3], q_current_deg[4], q_current_deg[5]

    fk_init = forward_kinematics(*q_current_deg)
    alpha = fk_init.orientation["alpha"]
    beta  = fk_init.orientation["beta"]
    gamma = fk_init.orientation["gamma"]

    def _step(theta_deg: float) -> ForwardKinematicsResult:
        """Calcula IK+FK para un ángulo theta en el arco."""
        nonlocal q_current_deg

        # Punto en el arco (coordenadas 2D del plano)
        arc_x = radius * np.cos(np.deg2rad(theta_deg)) + xc
        arc_y = radius * np.sin(np.deg2rad(theta_deg)) + yc

        px, py, pz = _unproject(np.array([arc_x, arc_y]), p_start, plane)

        fk_prev = forward_kinematics(*q_current_deg)
        ik = inverse_kinematics(
            px, py, pz,
            alpha, beta, gamma,
            fk_prev.A1, fk_prev.A2, fk_prev.A3,
        )

        fk = forward_kinematics(ik.q1_deg, ik.q2_deg, ik.q3_deg, q4, q5, q6)
        q_current_deg = fk.joints_deg
        return fk

    # ── Tramo 1: theta1 → theta2_adj ──────────────────────────────────────
    theta = theta1
    while (k > 0 and theta <= theta2_adj) or (k < 0 and theta >= theta2_adj):
        yield _step(theta)
        theta += k * step_deg

    # ── Tramo 2: theta2_adj → theta3_adj ──────────────────────────────────
    # Los ángulos se restauran al rango original antes del segundo tramo
    # (equivalente a los bloques if cambio==0/1 de CIR2D.m)
    theta = theta2_adj
    while (k > 0 and theta <= theta3_adj) or (k < 0 and theta >= theta3_adj):
        yield _step(theta)
        theta += k * step_deg

    # Frame final exacto en p_end
    yield _step(theta3)