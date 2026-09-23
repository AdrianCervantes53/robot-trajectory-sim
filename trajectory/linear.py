"""
linear.py
Linear trajectory generation in Cartesian space.

Linearly interpolates end-effector position between start and end points in N steps.
At each step, solves inverse kinematics to find joint angles and computes
forward kinematics to get the complete robot state. Orientation joints (q4, q5, q6)
are maintained constant throughout the movement.
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
    Generates a linear Cartesian trajectory as a sequence of FK results.

    Parameters
    ----------
    p_start : list[float]
        Initial position [px, py, pz].
    p_end : list[float]
        Final position [px, py, pz].
    q_current_deg : list[float]
        Current joint angles [q1..q6] in degrees.
    duration : float
        Total trajectory duration in seconds.
    dt : float
        Time step between points in seconds.

    Yields
    ------
    ForwardKinematicsResult
        Robot state at each point.

    Raises
    ------
    SingularityError
        If an intermediate point falls outside the reachable workspace.
    """
    po = np.array(p_start, dtype=float)
    pf = np.array(p_end,   dtype=float)

    # Constant orientation (q4, q5, q6) during linear motion
    q4, q5, q6 = q_current_deg[3], q_current_deg[4], q_current_deg[5]

    fk_init = forward_kinematics(*q_current_deg)
    alpha = fk_init.orientation["alpha"]
    beta  = fk_init.orientation["beta"]
    gamma = fk_init.orientation["gamma"]

    n = max(int(duration / dt), 1)
    delta = (pf - po) / n

    p = po.copy()
    for i in range(n + 1):
        fk_prev = forward_kinematics(*q_current_deg)
        ik = inverse_kinematics(
            p[0], p[1], p[2],
            alpha, beta, gamma,
            fk_prev.A1, fk_prev.A2, fk_prev.A3,
        )

        fk = forward_kinematics(
            ik.q1_deg, ik.q2_deg, ik.q3_deg,
            q4, q5, q6,
        )
        q_current_deg = fk.joints_deg

        yield fk

        if i < n:
            p = po + delta * (i + 1)
        else:
            p = pf.copy()
