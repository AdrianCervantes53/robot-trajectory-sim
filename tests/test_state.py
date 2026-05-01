"""
test_state.py
Tests unitarios para core/state.py.

Ejecutar con: pytest tests/test_state.py -v
"""

import json
import numpy as np
import pytest
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from core.state import RobotState
from kinematics.forward import forward_kinematics


class TestRobotStateInit:
    def test_from_home_es_valido(self):
        """from_home() debe retornar un estado con FK calculada."""
        state = RobotState.from_home()
        assert len(state.joints_deg) == 6
        assert "px" in state.position

    def test_from_home_angulos_cero(self):
        """En home todos los ángulos deben ser 0."""
        state = RobotState.from_home()
        for q in state.joints_deg:
            assert q == pytest.approx(0.0, abs=0.01)

    def test_from_home_posicion_correcta(self):
        """La posición home debe coincidir con FK directa."""
        state = RobotState.from_home()
        fk = forward_kinematics(0, 0, 0, 0, 0, 0)
        assert state.position["px"] == pytest.approx(fk.position["px"], abs=0.01)
        assert state.position["py"] == pytest.approx(fk.position["py"], abs=0.01)
        assert state.position["pz"] == pytest.approx(fk.position["pz"], abs=0.01)

    def test_defaults_hardware(self):
        """Valores por defecto del hardware deben ser seguros."""
        state = RobotState()
        assert state.gripper_open is True
        assert state.arduino_connected is False
        assert state.velocity_pct == 50.0


class TestApplyFKResult:
    def test_actualiza_joints(self):
        state = RobotState.from_home()
        fk = forward_kinematics(30, 20, 10, 0, 0, 0)
        state.apply_fk_result(fk)
        assert state.joints_deg[0] == pytest.approx(30.0, abs=0.1)
        assert state.joints_deg[1] == pytest.approx(20.0, abs=0.1)

    def test_actualiza_posicion(self):
        state = RobotState.from_home()
        fk = forward_kinematics(45, 30, 15, 0, 0, 0)
        state.apply_fk_result(fk)
        assert state.position["px"] == pytest.approx(fk.position["px"], abs=0.01)
        assert state.position["py"] == pytest.approx(fk.position["py"], abs=0.01)
        assert state.position["pz"] == pytest.approx(fk.position["pz"], abs=0.01)

    def test_actualiza_links(self):
        state = RobotState.from_home()
        fk = forward_kinematics(45, 30, 15, 0, 0, 0)
        state.apply_fk_result(fk)
        assert len(state.links) == 3

    def test_matrices_A_como_listas(self):
        """A1, A2, A3 deben guardarse como listas (JSON-safe), no como arrays."""
        state = RobotState.from_home()
        assert isinstance(state.A1, list)
        assert isinstance(state.A2, list)
        assert isinstance(state.A3, list)

    def test_A_np_devuelve_arrays(self):
        """Las propiedades A1_np/A2_np/A3_np deben devolver np.ndarray 4x4."""
        state = RobotState.from_home()
        for A in [state.A1_np, state.A2_np, state.A3_np]:
            assert isinstance(A, np.ndarray)
            assert A.shape == (4, 4)


class TestTrajectoryHistory:
    def test_record_agrega_punto(self):
        state = RobotState.from_home()
        state.record_trajectory_point()
        assert len(state.trajectory) == 1

    def test_record_guarda_posicion_actual(self):
        state = RobotState.from_home()
        px_expected = state.position["px"]
        state.record_trajectory_point()
        assert state.trajectory[0]["px"] == pytest.approx(px_expected, abs=0.01)

    def test_multiples_puntos(self):
        state = RobotState.from_home()
        for q1 in [0, 15, 30, 45]:
            fk = forward_kinematics(q1, 20, 10, 0, 0, 0)
            state.apply_fk_result(fk)
            state.record_trajectory_point()
        assert len(state.trajectory) == 4

    def test_clear_limpia_historial(self):
        state = RobotState.from_home()
        state.record_trajectory_point()
        state.record_trajectory_point()
        state.clear_trajectory()
        assert len(state.trajectory) == 0

    def test_record_es_copia(self):
        """Modificar el estado después de record no debe alterar el historial."""
        state = RobotState.from_home()
        state.record_trajectory_point()
        px_original = state.trajectory[0]["px"]

        fk = forward_kinematics(60, 30, 15, 0, 0, 0)
        state.apply_fk_result(fk)

        assert state.trajectory[0]["px"] == pytest.approx(px_original, abs=0.01)


class TestSerialization:
    def test_to_dict_tiene_claves_requeridas(self):
        state = RobotState.from_home()
        d = state.to_dict()
        for key in ["joints_deg", "position", "orientation", "links",
                    "transform", "trajectory", "gripper_open", "arduino_connected"]:
            assert key in d

    def test_to_json_es_string_valido(self):
        state = RobotState.from_home()
        raw = state.to_json()
        assert isinstance(raw, str)
        parsed = json.loads(raw)
        assert "joints_deg" in parsed

    def test_to_json_sin_numpy_types(self):
        """El JSON no debe contener tipos numpy — deben ser float/int nativos."""
        state = RobotState.from_home()
        fk = forward_kinematics(30, 20, 10, 0, 0, 0)
        state.apply_fk_result(fk)
        # Si hubiera numpy scalars esto lanzaría TypeError
        raw = json.loads(state.to_json())
        for q in raw["joints_deg"]:
            assert isinstance(q, (int, float))

    def test_position_list_es_lista_3(self):
        state = RobotState.from_home()
        pl = state.position_list
        assert len(pl) == 3
        assert isinstance(pl, list)