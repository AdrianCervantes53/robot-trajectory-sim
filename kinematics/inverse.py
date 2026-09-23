"""
inverse.py
Inverse kinematics for 6-DOF articulated robot.

Strategy:
- q1, q2, q3 -> geometric solution (arm position)
- q4, q5, q6 -> spherical wrist decoupling recalculating AR with
  newly found q1..q3, and desired rotation matrix R.

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


class SingularityError(Exception):
    """Raised when the target point is outside the workspace or causes a singularity."""
    pass


@dataclass
class InverseKinematicsResult:
    """
    Inverse kinematics computation result.

    Attributes
    ----------
    q1_deg .. q6_deg : float
        6 joint angles in degrees.
    joints_deg : list[float]
        6 joint angles as a list.
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
            float(self.q1_deg), float(self.q2_deg), float(self.q3_deg),
            float(self.q4_deg), float(self.q5_deg), float(self.q6_deg),
        ]


def _rotation_matrix(A: float, B: float, C: float) -> np.ndarray:
    """
    Constructs the target rotation matrix for the end-effector.

    Intrinsic ZYX convention:
        R = Rz(A) * Ry(B) * Rx(C)

    where:
        A -> rotation around Z (yaw / alpha)
        B -> rotation around Y (pitch / beta)
        C -> rotation around X (roll / gamma)

    Parameters
    ----------
    A, B, C : float
        Orientation angles in radians.
    """
    Rz = np.array([
        [np.cos(A), -np.sin(A), 0],
        [np.sin(A),  np.cos(A), 0],
        [0,          0,         1],
    ])
    Ry = np.array([
        [ np.cos(B), 0, np.sin(B)],
        [ 0,         1, 0        ],
        [-np.sin(B), 0, np.cos(B)],
    ])
    Rx = np.array([
        [1, 0,          0         ],
        [0, np.cos(C), -np.sin(C) ],
        [0, np.sin(C),  np.cos(C) ],
    ])
    return Rz @ Ry @ Rx


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
    Computes inverse kinematics for a target end-effector pose.

    Parameters
    ----------
    px, py, pz : float
        Target Cartesian coordinates.
    A, B, C : float
        Target orientation in radians (alpha, beta, gamma - ZYX convention).
    A1, A2, A3 : np.ndarray
        Current DH matrices (received for compatibility; AR is computed from target q1..q3).

    Returns
    -------
    InverseKinematicsResult

    Raises
    ------
    SingularityError
        If the target point is unreachable or causes a kinematic singularity.
    """
    try:
        # Position: q1, q2, q3 (geometric solution)
        r2 = px**2 + py**2
        R2 = r2 + (pz - L1)**2

        cos_q3 = (R2 - L2**2 - L3**2) / (2 * L2 * L3)

        # Reachability check
        if abs(cos_q3) > 1.0 + 1e-6:
            raise SingularityError(
                f"Point ({px:.2f}, {py:.2f}, {pz:.2f}) outside workspace (cos_q3={cos_q3:.4f} outside [-1, 1])."
            )

        cos_q3 = np.clip(cos_q3, -1.0, 1.0)
        sin_q3 = np.sqrt(1 - cos_q3**2)
        q3 = np.arctan2(sin_q3, cos_q3)

        q1 = np.arctan2(py, px)

        alfa = np.arctan2(pz - L1, np.sqrt(r2))
        beta = np.arctan2(L3 * sin_q3, L2 + L3 * cos_q3)

        q2 = alfa - beta
        if q2 < 0:
            # Alternative elbow configuration
            q2 = alfa + beta
            q3 = -np.arctan2(sin_q3, cos_q3)

        # Orientation: q4, q5, q6 (wrist decoupling)
        _A1 = dh_matrix(q1, L1, 0,  np.pi / 2)
        _A2 = dh_matrix(q2, 0,  L2, 0)
        _A3 = dh_matrix(q3, 0,  L3, 0)

        AR = _A1[:3, :3] @ _A2[:3, :3] @ _A3[:3, :3]

        # Target rotation matrix
        R = _rotation_matrix(A, B, C)

        # Relative wrist rotation: MT = AR^T * R
        MT = AR.T @ R

        # Analytical extraction for DH wrist sequence:
        #   A4 = dh(0,  0, 0,  q4)
        #   A5 = dh(q5, 0, 0, -pi/2)
        #   A6 = dh(q6, 0, 0,  0)
        q4 = np.arctan2(MT[2, 2], MT[1, 2])
        q5 = np.arctan2(-MT[0, 2], np.sqrt(MT[1, 2]**2 + MT[2, 2]**2))
        q6 = np.arctan2(-MT[0, 1], MT[0, 0])

        return InverseKinematicsResult(
            q1_deg=float(np.rad2deg(q1)),
            q2_deg=float(np.rad2deg(q2)),
            q3_deg=float(np.rad2deg(q3)),
            q4_deg=float(np.rad2deg(q4)),
            q5_deg=float(np.rad2deg(q5)),
            q6_deg=float(np.rad2deg(q6)),
        )

    except SingularityError:
        raise
    except Exception as exc:
        raise SingularityError(
            f"Point ({px:.2f}, {py:.2f}, {pz:.2f}) outside workspace or singularity detected."
        ) from exc
