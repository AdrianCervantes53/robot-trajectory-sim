"""
state.py
Global robot state model.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
import numpy as np

from kinematics.forward import forward_kinematics, ForwardKinematicsResult


@dataclass
class RobotState:
    """
    Complete state of the robot at any given instant.

    Attributes
    ----------
    joints_deg : list[float]
        Current joint angles [q1..q6] in degrees.
    position : dict
        End-effector Cartesian position {"px", "py", "pz"}.
    orientation : dict
        End-effector orientation {"alpha", "beta", "gamma"} in radians (ZYX).
    A1, A2, A3 : list[list[float]]
        DH 4x4 matrices for links 1-3.
    links : list[dict]
        Robot segments [{"from": [x,y,z], "to": [x,y,z]}, ...] for rendering.
    transform : list[list[float]]
        Full 4x4 homogeneous transformation matrix T.
    trajectory : list[dict]
        History of end-effector positions during trajectories.
    gripper_open : bool
        Gripper state. True = open, False = closed.
    arduino_connected : bool
        Whether an Arduino hardware device is connected.
    velocity_pct : float
        PTP trajectory speed percentage (0 to 100).
    trajectory_duration : float
        Duration for LIN/CIR trajectories in seconds.
    """

    # Kinematic state
    joints_deg: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    position: dict   = field(default_factory=lambda: {"px": 0.0, "py": 0.0, "pz": 0.0})
    orientation: dict = field(default_factory=lambda: {"alpha": 0.0, "beta": 0.0, "gamma": 0.0})
    A1: list = field(default_factory=lambda: np.eye(4).tolist())
    A2: list = field(default_factory=lambda: np.eye(4).tolist())
    A3: list = field(default_factory=lambda: np.eye(4).tolist())
    links: list   = field(default_factory=list)
    transform: list = field(default_factory=lambda: np.eye(4).tolist())

    # Trajectory history
    trajectory: list = field(default_factory=list)

    # Hardware and configuration
    gripper_open: bool      = True
    arduino_connected: bool = False
    velocity_pct: float     = 50.0
    trajectory_duration: float = 2.0

    @classmethod
    def from_home(cls) -> "RobotState":
        """Creates initial robot state at home position (all angles at zero)."""
        state = cls()
        state.apply_fk_result(forward_kinematics(0, 0, 0, 0, 0, 0))
        return state

    def apply_fk_result(self, fk: ForwardKinematicsResult) -> None:
        """Updates kinematic state from a ForwardKinematicsResult."""
        self.joints_deg  = fk.joints_deg
        self.position    = fk.position
        self.orientation = fk.orientation
        self.links       = fk.links
        self.transform   = fk.transform
        self.A1          = fk.A1.tolist()
        self.A2          = fk.A2.tolist()
        self.A3          = fk.A3.tolist()

    def record_trajectory_point(self) -> None:
        """Appends current end-effector position to trajectory history."""
        self.trajectory.append(dict(self.position))

    def clear_trajectory(self) -> None:
        """Clears recorded trajectory history."""
        self.trajectory.clear()

    def to_dict(self) -> dict:
        """Serializes full state to a JSON-compatible dictionary."""
        return {
            "joints_deg":   self.joints_deg,
            "position":     self.position,
            "orientation":  {k: float(v) for k, v in self.orientation.items()},
            "links":        self.links,
            "transform":    self.transform,
            "trajectory":   self.trajectory,
            "gripper_open": self.gripper_open,
            "arduino_connected": self.arduino_connected,
        }

    def to_json(self) -> str:
        """Serializes full state to a JSON string."""
        return json.dumps(self.to_dict(), default=float)

    @property
    def A1_np(self) -> np.ndarray:
        return np.array(self.A1)

    @property
    def A2_np(self) -> np.ndarray:
        return np.array(self.A2)

    @property
    def A3_np(self) -> np.ndarray:
        return np.array(self.A3)

    @property
    def position_list(self) -> list:
        return [self.position["px"], self.position["py"], self.position["pz"]]
