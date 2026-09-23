"""
ptp.py
Point-to-point (PTP) trajectory generation with 4th-order polynomial blend.

Motion profile has 3 symmetric stages:
  1. Acceleration   [0, T/4]    -> 4th order polynomial
  2. Cruise speed   [T/4, 3T/4] -> constant linear velocity
  3. Deceleration   [3T/4, T]   -> mirrored 4th order polynomial
"""

import numpy as np
from dataclasses import dataclass
from typing import Generator

from kinematics.forward import forward_kinematics, ForwardKinematicsResult

VEL_MAX: float = 2 * np.pi


@dataclass
class PTPCoeffs:
    """
    Polynomial profile coefficients for a single joint.
    """
    accel: np.ndarray
    vel_m: float
    vel_po2: float
    vel_dt: float
    decel: np.ndarray
    dt: float
    t2: float


def _poly4_coeffs(
    po: float, pf: float, vo: float, vf: float, T_seg: float
) -> np.ndarray:
    """
    Computes 4th-order polynomial coefficients for a segment.
    Returns coefficients in descending order [a4, a3, a2, a1, a0].
    """
    t1 = T_seg
    a0 = po
    a1 = vo
    a2 = 0.0
    a3 = (1 / (2 * t1**3)) * (20 * (pf - po) - (8 * vf + 12 * vo) * t1)
    a4 = (1 / (2 * t1**4)) * (30 * (po - pf) + (14 * vf + 16 * vo) * t1)
    return np.array([a4, a3, a2, a1, a0])


def _build_profile(po: float, pf: float, T: float) -> PTPCoeffs:
    """
    Builds the complete polynomial profile for a single joint.
    """
    dt = T * 0.25
    t1 = 2 * dt
    t2 = T - 2 * dt

    m = (pf - po) / (T - 2 * dt) if (T - 2 * dt) > 1e-10 else 0.0

    po2 = m * (t1 - dt) + po
    pf2 = m * (t2 - dt) + po

    accel = _poly4_coeffs(po, po2, 0.0, m, t1)
    decel = _poly4_coeffs(pf2, pf, m, 0.0, T - t2)

    return PTPCoeffs(
        accel=accel,
        vel_m=m,
        vel_po2=po2,
        vel_dt=dt,
        decel=decel,
        dt=t1,
        t2=t2,
    )


def _eval_profile(coeffs: PTPCoeffs, t: float) -> float:
    """Evaluates the polynomial profile at time t."""
    if t <= coeffs.dt:
        return float(np.polyval(coeffs.accel, t))
    elif t <= coeffs.t2:
        return coeffs.vel_m * (t - coeffs.vel_dt) + coeffs.vel_po2
    else:
        return float(np.polyval(coeffs.decel, t - coeffs.t2))


def ptp_trajectory(
    q_start_deg: list,
    q_end_deg: list,
    velocity_pct: float = 50.0,
    dt: float = 0.1,
) -> Generator[ForwardKinematicsResult, None, None]:
    """
    Generates a PTP trajectory as a sequence of FK results.

    Parameters
    ----------
    q_start_deg : list[float]
        Initial joint angles [q1..q6] in degrees.
    q_end_deg : list[float]
        Target joint angles [q1..q6] in degrees.
    velocity_pct : float
        Velocity percentage (0 to 100).
    dt : float
        Time step between points in seconds.

    Yields
    ------
    ForwardKinematicsResult
        Robot state at each time instant.
    """
    q_start = np.deg2rad(q_start_deg[:3])
    q_end   = np.deg2rad(q_end_deg[:3])
    q4, q5, q6 = q_start_deg[3], q_start_deg[4], q_start_deg[5]

    T_BASE = 3.0
    vel_factor = abs(velocity_pct - 100) / 100
    dismax = np.max(np.abs(q_end - q_start))
    dist_norm = dismax / np.pi if dismax > 1e-10 else 0.0
    T = T_BASE * vel_factor * dist_norm
    T = max(T, 0.1)

    profiles = [_build_profile(q_start[i], q_end[i], T) for i in range(3)]

    t = 0.0
    while True:
        t_clamped = min(t, T)
        q1 = np.rad2deg(_eval_profile(profiles[0], t_clamped))
        q2 = np.rad2deg(_eval_profile(profiles[1], t_clamped))
        q3 = np.rad2deg(_eval_profile(profiles[2], t_clamped))

        yield forward_kinematics(q1, q2, q3, q4, q5, q6)

        if t >= T:
            break
        t = min(t + dt, T)
