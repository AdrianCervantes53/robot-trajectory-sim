"""
inverse.py
Cinemática inversa del robot de 6 DOF.

MATLAB equivalente: fcinversa.m

Estrategia
----------
- q1, q2, q3 → solución geométrica (posición del EF)
- q4, q5, q6 → desacoplamiento de muñeca usando las matrices A1/A2/A3
  de la cinemática directa y la orientación deseada R.

Geometría del robot
-------------------
L1 = 5  (altura de la base)
L2 = 5  (eslabón 2)
L3 = 5  (eslabón 3)
"""

import numpy as np
from dataclasses import dataclass

L1: float = 5.0
L2: float = 5.0
L3: float = 5.0


class SingularityError(Exception):
    """Se lanza cuando el punto objetivo está fuera del espacio de trabajo."""
    pass


@dataclass
class InverseKinematicsResult:
    """
    Resultado de la cinemática inversa.

    Atributos
    ---------
    q1_deg .. q6_deg : float
        Ángulos de las 6 juntas en GRADOS.
    joints_deg : list[float]
        Los 6 ángulos como lista, en el mismo orden.
    """
    q1_deg: float
    q2_deg: float
    q3_deg: float
    q4_deg: float
    q5_deg: float
    q6_deg: float

    @property
    def joints_deg(self) -> list:
        return [
            self.q1_deg, self.q2_deg, self.q3_deg,
            self.q4_deg, self.q5_deg, self.q6_deg,
        ]


def _rotation_matrix(A: float, B: float, C: float) -> np.ndarray:
    """
    Construye la matriz de rotación deseada para el efector final.

    Usa la convención del MATLAB original: R = Rx(A) * Rz(B) * Ry(C)
    donde A=yaw (Z), B=pitch (Y), C=roll (X) — orden ZYX intrínseco.

    Parámetros
    ----------
    A, B, C : float
        Ángulos de orientación en RADIANES.
    """
    Rx = np.array([
        [np.cos(A), -np.sin(A), 0],
        [np.sin(A),  np.cos(A), 0],
        [0,          0,         1],
    ])
    Ry = np.array([
        [ np.cos(B), 0, np.sin(B)],
        [ 0,         1, 0        ],
        [-np.sin(B), 0, np.cos(B)],
    ])
    Rz = np.array([
        [1, 0,          0         ],
        [0, np.cos(C), -np.sin(C) ],
        [0, np.sin(C),  np.cos(C) ],
    ])
    return Rx @ Rz @ Ry


def inverse_kinematics(
    px: float,
    py: float,
    pz: float,
    A: float,
    B: float,
    C: float,
    A1: np.ndarray,
    A2: np.ndarray,
    A3: np.ndarray,
) -> InverseKinematicsResult:
    """
    Calcula la cinemática inversa para una pose deseada del efector final.

    Parámetros
    ----------
    px, py, pz : float
        Posición cartesiana deseada del efector final.
    A, B, C : float
        Orientación deseada en RADIANES (ángulos alpha, beta, gamma del EF).
    A1, A2, A3 : np.ndarray
        Matrices DH 4x4 del estado actual del robot (obtenidas de
        forward_kinematics). Necesarias para el desacoplamiento de muñeca.

    Retorna
    -------
    InverseKinematicsResult

    Lanza
    -----
    SingularityError
        Si el punto está fuera del espacio de trabajo o causa singularidad.
    """
    try:
        # ── Posición: q1, q2, q3 (solución geométrica) ────────────────────

        r2 = px**2 + py**2
        R2 = r2 + (pz - L1)**2

        cos_q3 = (R2 - L2**2 - L3**2) / (2 * L2 * L3)

        # Validar alcanzabilidad antes del clip numérico
        if abs(cos_q3) > 1.0 + 1e-6:
            raise SingularityError(
                f"Punto ({px:.2f}, {py:.2f}, {pz:.2f}) fuera del espacio de trabajo "
                f"(cos_q3={cos_q3:.4f} fuera de [-1, 1])."
            )

        # Clamp numérico menor para errores de punto flotante (~1e-7)
        cos_q3 = np.clip(cos_q3, -1.0, 1.0)

        sin_q3 = np.sqrt(1 - cos_q3**2)
        q3 = np.arctan2(sin_q3, cos_q3)

        q1 = np.arctan2(py, px)

        alfa = np.arctan2(pz - L1, np.sqrt(r2))
        beta = np.arctan2(L3 * sin_q3, L2 + L3 * cos_q3)

        q2 = alfa - beta
        if q2 < 0:
            # Configuración alternativa (codo arriba/abajo)
            q2 = alfa + beta
            q3 = -np.arctan2(sin_q3, cos_q3)

        # ── Orientación: q4, q5, q6 (desacoplamiento de muñeca) ───────────

        # Submatrices de rotación 3x3 de las transformaciones acumuladas
        Aa = A1[:3, :3]
        Ab = A2[:3, :3]
        Ac = A3[:3, :3]

        # Rotación acumulada del brazo (juntas 1-3)
        AR = Aa @ Ab @ Ac

        # Rotación deseada del efector
        R = _rotation_matrix(A, B, C)

        # Rotación relativa que deben cubrir las juntas de muñeca (4-6)
        MT = AR.T @ R

        q4 = np.arctan2(MT[2, 1], MT[1, 1])
        q5 = np.arctan2(-MT[0, 1], np.sqrt(MT[1, 1]**2 + MT[2, 1]**2))
        q6 = np.arctan2(MT[0, 2], MT[0, 0])

        return InverseKinematicsResult(
            q1_deg=np.rad2deg(q1),
            q2_deg=np.rad2deg(q2),
            q3_deg=np.rad2deg(q3),
            q4_deg=np.rad2deg(q4),
            q5_deg=np.rad2deg(q5),
            q6_deg=np.rad2deg(q6),
        )

    except Exception as exc:
        raise SingularityError(
            f"Punto ({px:.2f}, {py:.2f}, {pz:.2f}) fuera del espacio de trabajo "
            f"o singularidad detectada."
        ) from exc