"""
forward.py
Forward kinematics for 6-DOF articulated robot.

Robot Geometry:
L1 = 5.0 (base height)
L2 = 5.0 (link 2 length)
L3 = 5.0 (link 3 length)
"""

import numpy as np
from dataclasses import dataclass

from kinematics.dh import dh_matrix

L1: float = 5.0
L2: float = 5.0
L3: float = 5.0


@dataclass
class ForwardKinematicsResult:
    """
    Forward kinematics computation result.

    Attributes
    ----------
    joints_deg : list[float]
        6 joint angles in degrees.
    position : dict
        End-effector position {"px", "py", "pz"}.
    orientation : dict
        End-effector orientation in ZYX Euler angles {"alpha", "beta", "gamma"} in radians.
    links : list[dict]
        Robot segments for rendering [{"from": [x,y,z], "to": [x,y,z]}, ...].
    transform : list[list[float]]
        Full 4x4 homogeneous transformation matrix T.
    A1, A2, A3 : np.ndarray
        Link DH transformation matrices.
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
    Computes forward kinematics for the 6-DOF robot.

    Parameters
    ----------
    q1_deg .. q6_deg : float
        Joint angles in degrees.

    Returns
    -------
    ForwardKinematicsResult
        Position, orientation, link segments, and DH matrices.
    """
    q1 = np.deg2rad(q1_deg)
    q2 = np.deg2rad(q2_deg)
    q3 = np.deg2rad(q3_deg)
    q4 = np.deg2rad(q4_deg)
    q5 = np.deg2rad(q5_deg)
    q6 = np.deg2rad(q6_deg)

    # Individual DH matrices
    A1 = dh_matrix(q1, L1, 0,  np.pi / 2)
    A2 = dh_matrix(q2, 0,  L2, 0)
    A3 = dh_matrix(q3, 0,  L3, 0)
    A4 = dh_matrix(0,  0,  0,  q4)
    A5 = dh_matrix(q5, 0,  0, -np.pi / 2)
    A6 = dh_matrix(q6, 0,  0,  0)

    # Accumulated transformations
    A21 = A1 @ A2
    A321 = A21 @ A3
    T = A321 @ A4 @ A5 @ A6

    # Joint positions for drawing link segments
    x1, y1, z1 = A1[0, 3], A1[1, 3], A1[2, 3]
    x2, y2, z2 = A21[0, 3], A21[1, 3], A21[2, 3]
    px, py, pz = T[0, 3], T[1, 3], T[2, 3]

    # End-effector orientation (ZYX Euler angles: T = Rz(alpha)*Ry(beta)*Rx(gamma))
    alpha = np.arctan2(T[1, 0], T[0, 0])                                 # yaw   (Z)
    beta  = np.arctan2(-T[2, 0], np.sqrt(T[0, 0]**2 + T[1, 0]**2))      # pitch (Y)
    gamma = np.arctan2(T[2, 1], T[2, 2])                                  # roll  (X)

    return ForwardKinematicsResult(
        joints_deg=[
            round(float(np.rad2deg(q1)), 2),
            round(float(np.rad2deg(q2)), 2),
            round(float(np.rad2deg(q3)), 2),
            round(float(np.rad2deg(q4)), 2),
            round(float(np.rad2deg(q5)), 2),
            round(float(np.rad2deg(q6)), 2),
        ],
        position={"px": round(float(px), 2), "py": round(float(py), 2), "pz": round(float(pz), 2)},
        orientation={"alpha": float(alpha), "beta": float(beta), "gamma": float(gamma)},
        links=[
            {"from": [0.0,  0.0,  0.0], "to": [float(x1), float(y1), float(z1)]},
            {"from": [float(x1), float(y1), float(z1)], "to": [float(x2), float(y2), float(z2)]},
            {"from": [float(x2), float(y2), float(z2)], "to": [float(px), float(py), float(pz)]},
        ],
        transform=T.tolist(),
        A1=A1,
        A2=A2,
        A3=A3,
    )
