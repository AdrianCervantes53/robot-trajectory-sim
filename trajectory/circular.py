"""
circular.py
Circular trajectory generation in Cartesian space (3 points define the arc).

Supported planes:
  plane=1 -> XY (varies px, py; constant pz)
  plane=2 -> XZ (varies px, pz; constant py)
  plane=3 -> YZ (varies py, pz; constant px)
"""

import numpy as np
from typing import Generator, Optional

from kinematics.forward import forward_kinematics, ForwardKinematicsResult
from kinematics.inverse import inverse_kinematics, SingularityError


def _circumcenter(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray):
    """
    Computes center and radius of the circle passing through 3 2D points.

    Returns
    -------
    (center, radius) : (np.ndarray shape (2,), float)

    Raises
    ------
    ValueError
        If points are collinear.
    """
    ax, ay = p1
    bx, by = p2
    cx, cy = p3

    D = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(D) < 1e-10:
        raise ValueError("Points are collinear (colineales) - cannot define a unique circle.")

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
    Determines direction (+/-1) and adjusted angles to traverse
    the arc theta1 -> theta2 -> theta3 through the intermediate point.

    Returns
    -------
    (k, theta2_adj, theta3_adj) where k=+1 counterclockwise, k=-1 clockwise.
    """
    t1, t2, t3 = theta1, theta2, theta3

    if t3 > t2 > t1:
        return +1, t2, t3
    elif t1 > t2 > t3:
        return -1, t2, t3
    elif t1 > t3 > t2:
        return +1, t2 + 360, t3
    elif t2 > t3 > t1:
        return -1, t2 - 360, t3
    elif t2 > t1 > t3:
        return +1, t2, t3 + 360
    else:
        return -1, t2, t3 - 360


def _project(p_3d: list, plane: int):
    """Extracts the 2 active planar coordinates according to plane."""
    px, py, pz = p_3d
    if plane == 1:
        return np.array([px, py])
    elif plane == 2:
        return np.array([px, pz])
    else:
        return np.array([py, pz])


def _unproject(xy: np.ndarray, p_ref: list, plane: int) -> tuple:
    """Reconstructs (px, py, pz) from the 2 active planar coordinates and reference position."""
    px_ref, py_ref, pz_ref = p_ref
    if plane == 1:
        return xy[0], xy[1], pz_ref
    elif plane == 2:
        return xy[0], py_ref, xy[1]
    else:
        return px_ref, xy[0], xy[1]


def circular_trajectory(
    p_start: list,
    p_mid: list,
    p_end: list,
    q_current_deg: list,
    plane: int = 1,
    duration: float = 2.0,
    dt: float = 0.05,
    step_deg: Optional[float] = None,
) -> Generator[ForwardKinematicsResult, None, None]:
    """
    Generates a circular Cartesian trajectory as a sequence of FK results.

    Parameters
    ----------
    p_start : list[float]
        Start position [px, py, pz].
    p_mid : list[float]
        Intermediate waypoint [px, py, pz].
    p_end : list[float]
        End position [px, py, pz].
    q_current_deg : list[float]
        Current joint angles [q1..q6] in degrees.
    plane : int
        Motion plane: 1=XY, 2=XZ, 3=YZ.
    duration : float
        Total trajectory duration in seconds.
    dt : float
        Time step between frames in seconds.
    step_deg : float, optional
        Angular step in degrees. Overrides duration/dt division if provided.

    Yields
    ------
    ForwardKinematicsResult
        Robot state at each point along the arc.

    Raises
    ------
    ValueError
        If the 3 points are collinear.
    SingularityError
        If an arc point falls outside the workspace.
    """
    pt1 = _project(p_start, plane)
    pt2 = _project(p_mid,   plane)
    pt3 = _project(p_end,   plane)

    center, radius = _circumcenter(pt1, pt2, pt3)
    xc, yc = center

    theta1 = np.rad2deg(np.arctan2(pt1[1] - yc, pt1[0] - xc))
    theta2 = np.rad2deg(np.arctan2(pt2[1] - yc, pt2[0] - xc))
    theta3 = np.rad2deg(np.arctan2(pt3[1] - yc, pt3[0] - xc))

    k, theta2_adj, theta3_adj = _arc_direction(theta1, theta2, theta3)

    arc_total_deg = abs(theta2_adj - theta1) + abs(theta3_adj - theta2_adj)
    if arc_total_deg < 1e-6:
        arc_total_deg = 360.0

    if step_deg is not None and step_deg > 0:
        step = float(step_deg)
    else:
        n_steps = max(int(duration / dt), 1)
        step = arc_total_deg / n_steps

    q4, q5, q6 = q_current_deg[3], q_current_deg[4], q_current_deg[5]

    fk_init = forward_kinematics(*q_current_deg)
    alpha = fk_init.orientation["alpha"]
    beta  = fk_init.orientation["beta"]
    gamma = fk_init.orientation["gamma"]

    def _step(theta_deg: float) -> ForwardKinematicsResult:
        nonlocal q_current_deg

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

    # Segment 1: theta1 -> theta2_adj
    theta = theta1
    while (k > 0 and theta <= theta2_adj) or (k < 0 and theta >= theta2_adj):
        yield _step(theta)
        theta += k * step

    # Segment 2: theta2_adj -> theta3_adj
    theta = theta2_adj
    while (k > 0 and theta <= theta3_adj) or (k < 0 and theta >= theta3_adj):
        yield _step(theta)
        theta += k * step

    # Final frame at p_end
    yield _step(theta3)
