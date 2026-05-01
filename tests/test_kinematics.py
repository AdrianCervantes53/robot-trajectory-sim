"""
test_kinematics.py
Tests unitarios para utils.py, dh.py y forward.py.

Ejecutar con: pytest tests/test_kinematics.py -v
"""

import numpy as np
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from kinematics.dh import dh_matrix
from kinematics.forward import forward_kinematics

# ──────────────────────────────────────────────
# dh.py
# ──────────────────────────────────────────────

class TestDHMatrix:
    def test_forma_matriz(self):
        A = dh_matrix(0, 0, 0, 0)
        assert A.shape == (4, 4)

    def test_identidad_con_ceros(self):
        """Con todos los parámetros en cero la matriz debe ser identidad."""
        A = dh_matrix(0, 0, 0, 0)
        np.testing.assert_allclose(A, np.eye(4), atol=1e-10)

    def test_ultima_fila_siempre_0001(self):
        """La última fila homogénea debe ser [0, 0, 0, 1] siempre."""
        for theta in [0, np.pi / 4, np.pi / 2, np.pi]:
            A = dh_matrix(theta, 2, 3, np.pi / 3)
            np.testing.assert_allclose(A[3], [0, 0, 0, 1], atol=1e-10)

    def test_traslacion_d(self):
        """Con theta=0, a=0, alpha=0 solo debe trasladar en Z por d."""
        A = dh_matrix(0, 7, 0, 0)
        assert A[2, 3] == pytest.approx(7.0)
        assert A[0, 3] == pytest.approx(0.0)
        assert A[1, 3] == pytest.approx(0.0)

    def test_traslacion_a(self):
        """Con theta=0, d=0, alpha=0 solo debe trasladar en X por a."""
        A = dh_matrix(0, 0, 4, 0)
        assert A[0, 3] == pytest.approx(4.0)
        assert A[2, 3] == pytest.approx(0.0)


# ──────────────────────────────────────────────
# forward.py
# ──────────────────────────────────────────────

class TestForwardKinematics:
    def test_posicion_home(self):
        """
        En posición home (todos en cero) los eslabones 2 y 3 apuntan
        horizontalmente (eje X), por lo que:
          - pz = L1 = 5  (solo la altura de la base)
          - py = 0
          - px = L2 + L3 = 10
        """
        result = forward_kinematics(0, 0, 0, 0, 0, 0)
        assert result.position["pz"] == pytest.approx(5.0, abs=0.01)
        assert result.position["py"] == pytest.approx(0.0, abs=0.01)
        assert result.position["px"] == pytest.approx(10.0, abs=0.01)

    def test_estructura_links(self):
        """Debe retornar exactamente 3 segmentos con claves 'from' y 'to'."""
        result = forward_kinematics(0, 0, 0, 0, 0, 0)
        assert len(result.links) == 3
        for link in result.links:
            assert "from" in link
            assert "to" in link
            assert len(link["from"]) == 3
            assert len(link["to"]) == 3

    def test_estructura_joints_deg(self):
        """Debe retornar exactamente 6 ángulos."""
        result = forward_kinematics(10, 20, 30, 40, 50, 60)
        assert len(result.joints_deg) == 6

    def test_transform_es_4x4(self):
        """La matriz de transformación homogénea debe ser 4x4."""
        result = forward_kinematics(0, 0, 0, 0, 0, 0)
        T = np.array(result.transform)
        assert T.shape == (4, 4)

    def test_transform_ultima_fila(self):
        """La última fila de T debe ser [0, 0, 0, 1]."""
        result = forward_kinematics(30, 45, 60, 0, 0, 0)
        T = np.array(result.transform)
        np.testing.assert_allclose(T[3], [0, 0, 0, 1], atol=1e-10)

    def test_matrices_A_disponibles(self):
        """El resultado debe exponer A1, A2, A3 como arrays 4x4."""
        result = forward_kinematics(0, 0, 0, 0, 0, 0)
        for A in [result.A1, result.A2, result.A3]:
            assert A.shape == (4, 4)

    def test_link_base_inicia_en_origen(self):
        """El primer segmento siempre arranca en [0, 0, 0]."""
        result = forward_kinematics(45, 30, 15, 0, 0, 0)
        assert result.links[0]["from"] == [0.0, 0.0, 0.0]

    def test_links_continuos(self):
        """El 'to' de un segmento debe coincidir con el 'from' del siguiente."""
        result = forward_kinematics(20, 30, 10, 0, 0, 0)
        assert result.links[0]["to"] == pytest.approx(result.links[1]["from"], abs=1e-6)
        assert result.links[1]["to"] == pytest.approx(result.links[2]["from"], abs=1e-6)

    def test_ef_coincide_con_transform(self):
        """La posición del EF debe coincidir con la columna de traslación de T."""
        result = forward_kinematics(30, 45, 0, 0, 0, 0)
        T = np.array(result.transform)
        assert result.position["px"] == pytest.approx(T[0, 3], abs=0.01)
        assert result.position["py"] == pytest.approx(T[1, 3], abs=0.01)
        assert result.position["pz"] == pytest.approx(T[2, 3], abs=0.01)