"""
state.py
Estado global del robot — reemplaza el workspace de MATLAB.

En el MATLAB original, todas las funciones compartían estado a través de
evalin/assignin sobre el workspace base:

    Q  = evalin('base', 'Q')       → ángulos actuales (grados)
    P  = evalin('base', 'P')       → posición + orientación del EF
    A1 = evalin('base', 'A1')      → matrices DH intermedias
    ...
    assignin('base', 'Q', Q)       → escritura de vuelta

RobotState centraliza todo ese estado en un dataclass mutable con un
método de actualización único. Es el único objeto que la API manipula.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Optional
import numpy as np

from kinematics.forward import forward_kinematics, ForwardKinematicsResult


@dataclass
class RobotState:
    """
    Estado completo del robot en un instante dado.

    Atributos
    ---------
    joints_deg : list[float]
        Ángulos actuales [q1..q6] en GRADOS. Equivale a Q en MATLAB.
    position : dict
        Posición cartesiana del EF {"px", "py", "pz"}. Parte de P.
    orientation : dict
        Orientación del EF {"alpha", "beta", "gamma"} en radianes. Parte de P.
    A1, A2, A3 : list[list[float]]
        Matrices DH 4x4 de los eslabones 1-3. Necesarias para IK.
    links : list[dict]
        Segmentos del robot [{"from": [x,y,z], "to": [x,y,z]}, ...].
        Listos para serializar y enviar al frontend (Three.js).
    transform : list[list[float]]
        Matriz de transformación homogénea T completa (4x4).
    trajectory : list[dict]
        Historial de posiciones del EF durante trayectorias.
        Equivale a PEF en MATLAB. Cada elemento es {"px", "py", "pz"}.
    gripper_open : bool
        Estado del gripper. True = abierto, False = cerrado.
        Equivale a g en MATLAB (0=cerrado, 1=abierto, pero invertido).
    arduino_connected : bool
        Si hay un Arduino conectado. Equivale a ardno en MATLAB.
    velocity_pct : float
        Velocidad de trayectoria PTP 0-100. Equivale a Vel en MATLAB.
    trajectory_duration : float
        Duración de trayectorias LIN/CIR en segundos. Equivale a T en MATLAB.
    """

    # Estado cinemático
    joints_deg: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    position: dict   = field(default_factory=lambda: {"px": 0.0, "py": 0.0, "pz": 0.0})
    orientation: dict = field(default_factory=lambda: {"alpha": 0.0, "beta": 0.0, "gamma": 0.0})
    A1: list = field(default_factory=lambda: np.eye(4).tolist())
    A2: list = field(default_factory=lambda: np.eye(4).tolist())
    A3: list = field(default_factory=lambda: np.eye(4).tolist())
    links: list   = field(default_factory=list)
    transform: list = field(default_factory=lambda: np.eye(4).tolist())

    # Historial de trayectoria
    trajectory: list = field(default_factory=list)

    # Hardware y configuración
    gripper_open: bool      = True
    arduino_connected: bool = False
    velocity_pct: float     = 50.0
    trajectory_duration: float = 2.0

    # ─────────────────────────────────────────────────────────────
    # Fábrica
    # ─────────────────────────────────────────────────────────────

    @classmethod
    def from_home(cls) -> "RobotState":
        """
        Crea el estado inicial del robot en posición home
        (todos los ángulos en cero) y calcula la cinemática directa.
        """
        state = cls()
        state.apply_fk_result(forward_kinematics(0, 0, 0, 0, 0, 0))
        return state

    # ─────────────────────────────────────────────────────────────
    # Actualización
    # ─────────────────────────────────────────────────────────────

    def apply_fk_result(self, fk: ForwardKinematicsResult) -> None:
        """
        Actualiza el estado completo a partir de un ForwardKinematicsResult.
        Es el único punto de escritura del estado cinemático — equivalente
        a todos los assignin('base', ...) que había en fcdirecta.m.
        """
        self.joints_deg  = fk.joints_deg
        self.position    = fk.position
        self.orientation = fk.orientation
        self.links       = fk.links
        self.transform   = fk.transform
        self.A1          = fk.A1.tolist()
        self.A2          = fk.A2.tolist()
        self.A3          = fk.A3.tolist()

    def record_trajectory_point(self) -> None:
        """
        Agrega la posición actual del EF al historial de trayectoria.
        Equivale al array PEF en MATLAB.
        """
        self.trajectory.append(dict(self.position))

    def clear_trajectory(self) -> None:
        """Limpia el historial de trayectoria. Equivale a l=2 en MATLAB."""
        self.trajectory.clear()

    # ─────────────────────────────────────────────────────────────
    # Serialización
    # ─────────────────────────────────────────────────────────────

    def to_dict(self) -> dict:
        """
        Serializa el estado completo a un dict JSON-safe.
        Es lo que el WebSocket envía al frontend en cada frame.
        """
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
        """Versión JSON string de to_dict(). Lista para enviar por WebSocket."""
        return json.dumps(self.to_dict(), default=float)

    # ─────────────────────────────────────────────────────────────
    # Propiedades de conveniencia
    # ─────────────────────────────────────────────────────────────

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
        """Posición como lista [px, py, pz]. Útil para pasarla a trayectorias."""
        return [self.position["px"], self.position["py"], self.position["pz"]]