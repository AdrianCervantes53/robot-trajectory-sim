"""
dh.py
Matriz de transformación homogénea Denavit-Hartenberg.

MATLAB equivalente: fDH.m
"""

import numpy as np


def dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """
    Genera la matriz de transformación homogénea 4x4 usando parámetros DH estándar.

    Parámetros
    ----------
    theta : float
        Ángulo de junta (radianes).
    d : float
        Desplazamiento a lo largo del eje Z anterior.
    a : float
        Longitud del eslabón (distancia entre ejes Z).
    alpha : float
        Ángulo de torsión entre ejes Z (radianes).

    Retorna
    -------
    np.ndarray
        Matriz de transformación homogénea 4x4.
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct,  -ca * st,  sa * st,  a * ct],
        [st,   ca * ct, -sa * ct,  a * st],
        [0,    sa,       ca,       d     ],
        [0,    0,        0,        1     ],
    ])