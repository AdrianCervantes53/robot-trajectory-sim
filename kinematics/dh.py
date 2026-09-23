"""
dh.py
Denavit-Hartenberg 4x4 homogeneous transformation matrix.
"""

import numpy as np


def dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """
    Generates standard 4x4 DH homogeneous transformation matrix.

    Parameters
    ----------
    theta : float
        Joint angle in radians.
    d : float
        Offset along previous Z axis.
    a : float
        Link length (distance between Z axes).
    alpha : float
        Twist angle between Z axes in radians.

    Returns
    -------
    np.ndarray
        4x4 homogeneous transformation matrix.
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
