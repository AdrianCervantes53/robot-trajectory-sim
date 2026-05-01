"""
test_inverse.py
Tests unitarios para inverse.py.

Estrategia principal: round-trip test.
  1. Dado un conjunto de ángulos, calcular FK → obtener posición.
  2. Pasar esa posición a IK → obtener ángulos de vuelta.
  3. Pasar esos ángulos de vuelta a FK → la posición debe ser la misma.

Esto valida IK + FK juntos sin necesitar valores hardcodeados.

Ejecutar con: pytest tests/test_inverse.py -v
"""

import numpy as np
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

from kinematics.forward import forward_kinematics
from kinematics.inverse import inverse_kinematics, SingularityError


def round_trip(q1, q2, q3, q4=0.0, q5=0.0, q6=0.0, tol=0.1):
    """
    Helper: FK → IK → FK y compara posiciones.
    Retorna (pos_original, pos_recalculada).
    """
    fk = forward_kinematics(q1, q2, q3, q4, q5, q6)
    pos = fk.position

    ik = inverse_kinematics(
        pos["px"], pos["py"], pos["pz"],
        fk.orientation["alpha"],
        fk.orientation["beta"],
        fk.orientation["gamma"],
        fk.A1, fk.A2, fk.A3,
    )

    fk2 = forward_kinematics(*ik.joints_deg)
    pos2 = fk2.position

    assert pos2["px"] == pytest.approx(pos["px"], abs=tol), f"px: {pos['px']} → {pos2['px']}"
    assert pos2["py"] == pytest.approx(pos["py"], abs=tol), f"py: {pos['py']} → {pos2['py']}"
    assert pos2["pz"] == pytest.approx(pos["pz"], abs=tol), f"pz: {pos['pz']} → {pos2['pz']}"

    return pos, pos2


class TestInverseKinematics:

    def test_round_trip_posicion_simple(self):
        """Configuración básica sin rotación."""
        round_trip(45, 30, 20)

    def test_round_trip_q1_negativo(self):
        """q1 negativo → robot apunta en dirección opuesta en XY."""
        round_trip(-45, 30, 20)

    def test_round_trip_q2_positivo(self):
        """q2 positivo levanta el brazo."""
        round_trip(0, 45, 0)

    def test_round_trip_varias_configuraciones(self):
        """Múltiples configuraciones del brazo."""
        configs = [
            (30, 20, 10),
            (90, 30, 15),
            (10, 10, 10),
            (60, 40, -10),
            (0,  30,  30),
        ]
        for cfg in configs:
            round_trip(*cfg)

    def test_resultado_tiene_6_angulos(self):
        """El resultado debe exponer exactamente 6 ángulos."""
        fk = forward_kinematics(30, 20, 10, 0, 0, 0)
        ik = inverse_kinematics(
            fk.position["px"], fk.position["py"], fk.position["pz"],
            fk.orientation["alpha"], fk.orientation["beta"], fk.orientation["gamma"],
            fk.A1, fk.A2, fk.A3,
        )
        assert len(ik.joints_deg) == 6

    def test_singularidad_fuera_de_espacio(self):
        """Un punto inalcanzable debe lanzar SingularityError."""
        fk = forward_kinematics(0, 0, 0, 0, 0, 0)
        with pytest.raises(SingularityError):
            inverse_kinematics(
                999, 999, 999,          # muy lejos del robot
                0, 0, 0,
                fk.A1, fk.A2, fk.A3,
            )

    def test_angulos_son_floats(self):
        """Todos los ángulos del resultado deben ser floats."""
        fk = forward_kinematics(45, 30, 15, 0, 0, 0)
        ik = inverse_kinematics(
            fk.position["px"], fk.position["py"], fk.position["pz"],
            fk.orientation["alpha"], fk.orientation["beta"], fk.orientation["gamma"],
            fk.A1, fk.A2, fk.A3,
        )
        for angle in ik.joints_deg:
            assert isinstance(angle, float)