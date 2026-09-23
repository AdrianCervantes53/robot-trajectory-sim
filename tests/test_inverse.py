"""
test_inverse.py
Tests unitarios para inverse.py.

Estrategia principal: round-trip test.
  1. Dado un conjunto de Ã¡ngulos, calcular FK â†’ obtener posiciÃ³n.
  2. Pasar esa posiciÃ³n a IK â†’ obtener Ã¡ngulos de vuelta.
  3. Pasar esos Ã¡ngulos de vuelta a FK â†’ la posiciÃ³n debe ser la misma.

Esto valida IK + FK juntos sin necesitar valores hardcodeados.

Ejecutar con: pytest tests/test_inverse.py -v
"""

import numpy as np
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from kinematics.forward import forward_kinematics
from kinematics.inverse import inverse_kinematics, SingularityError


def round_trip(q1, q2, q3, q4=0.0, q5=0.0, q6=0.0, tol=0.1):
    """
    Helper: FK â†’ IK â†’ FK y compara posiciones.
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

    assert pos2["px"] == pytest.approx(pos["px"], abs=tol), f"px: {pos['px']} â†’ {pos2['px']}"
    assert pos2["py"] == pytest.approx(pos["py"], abs=tol), f"py: {pos['py']} â†’ {pos2['py']}"
    assert pos2["pz"] == pytest.approx(pos["pz"], abs=tol), f"pz: {pos['pz']} â†’ {pos2['pz']}"

    return pos, pos2


class TestInverseKinematics:

    def test_round_trip_posicion_simple(self):
        """ConfiguraciÃ³n bÃ¡sica sin rotaciÃ³n."""
        round_trip(45, 30, 20)

    def test_round_trip_q1_negativo(self):
        """q1 negativo â†’ robot apunta en direcciÃ³n opuesta en XY."""
        round_trip(-45, 30, 20)

    def test_round_trip_q2_positivo(self):
        """q2 positivo levanta el brazo."""
        round_trip(0, 45, 0)

    def test_round_trip_varias_configuraciones(self):
        """MÃºltiples configuraciones del brazo."""
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
        """El resultado debe exponer exactamente 6 Ã¡ngulos."""
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
        """Todos los Ã¡ngulos del resultado deben ser floats."""
        fk = forward_kinematics(45, 30, 15, 0, 0, 0)
        ik = inverse_kinematics(
            fk.position["px"], fk.position["py"], fk.position["pz"],
            fk.orientation["alpha"], fk.orientation["beta"], fk.orientation["gamma"],
            fk.A1, fk.A2, fk.A3,
        )
        for angle in ik.joints_deg:
            assert isinstance(angle, float)
    def test_round_trip_con_orientacion_muneca(self):
        """Verifica que q4, q5, q6 no nulos recuperan la pose y orientacion completa."""
        configs = [
            (30, 20, 10, 15, 25, 35),
            (45, 30, 20, -20, 40, -50),
            (10, 40, 15, 30, 45, 60),
            (60, 30, -10, -15, -30, 45),
        ]
        for cfg in configs:
            fk1 = forward_kinematics(*cfg)
            ik = inverse_kinematics(
                fk1.position["px"], fk1.position["py"], fk1.position["pz"],
                fk1.orientation["alpha"], fk1.orientation["beta"], fk1.orientation["gamma"],
                fk1.A1, fk1.A2, fk1.A3,
            )
            fk2 = forward_kinematics(*ik.joints_deg)
            # Validar posicion
            assert fk2.position["px"] == pytest.approx(fk1.position["px"], abs=0.05)
            assert fk2.position["py"] == pytest.approx(fk1.position["py"], abs=0.05)
            assert fk2.position["pz"] == pytest.approx(fk1.position["pz"], abs=0.05)
            # Validar matriz de rotacion completa 3x3
            T1 = np.array(fk1.transform)
            T2 = np.array(fk2.transform)
            np.testing.assert_allclose(T1[:3, :3], T2[:3, :3], atol=1e-3)
