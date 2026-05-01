"""
test_ptp.py
Tests unitarios para trajectory/ptp.py.

Ejecutar con: pytest tests/test_ptp.py -v
"""

import numpy as np
import pytest
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from trajectory.ptp import ptp_trajectory, _build_profile, _eval_profile


class TestPTPProfile:
    """Tests del perfil polinomial por junta (_build_profile / _eval_profile)."""

    def test_inicia_en_posicion_inicial(self):
        """El perfil debe empezar exactamente en po."""
        p = _build_profile(0.0, 1.0, 2.0)
        assert _eval_profile(p, 0.0) == pytest.approx(0.0, abs=1e-6)

    def test_termina_en_posicion_final(self):
        """El perfil debe terminar exactamente en pf."""
        p = _build_profile(0.0, 1.0, 2.0)
        assert _eval_profile(p, 2.0) == pytest.approx(1.0, abs=1e-6)

    def test_posicion_estacionaria(self):
        """Si po == pf el perfil debe ser constante."""
        p = _build_profile(1.5, 1.5, 2.0)
        for t in [0.0, 0.5, 1.0, 1.5, 2.0]:
            assert _eval_profile(p, t) == pytest.approx(1.5, abs=1e-6)

    def test_monotono_hacia_adelante(self):
        """Movimiento positivo: el perfil nunca debe retroceder significativamente."""
        p = _build_profile(0.0, 2.0, 3.0)
        vals = [_eval_profile(p, t) for t in np.linspace(0, 3.0, 50)]
        diffs = np.diff(vals)
        # Permitir pequeñas oscilaciones numéricas en los extremos
        assert all(d >= -0.01 for d in diffs)

    def test_negativo_po_mayor_pf(self):
        """Movimiento negativo: pf < po."""
        p = _build_profile(1.0, 0.0, 2.0)
        assert _eval_profile(p, 0.0) == pytest.approx(1.0, abs=1e-6)
        assert _eval_profile(p, 2.0) == pytest.approx(0.0, abs=1e-6)


class TestPTPTrajectory:
    """Tests de la trayectoria PTP completa."""

    def _collect(self, *args, **kwargs):
        """Helper: recolecta todos los frames en una lista."""
        return list(ptp_trajectory(*args, **kwargs))

    def test_emite_al_menos_un_frame(self):
        frames = self._collect([0, 0, 0, 0, 0, 0], [45, 30, 20, 0, 0, 0])
        assert len(frames) >= 1

    def test_primer_frame_cerca_del_inicio(self):
        """El primer frame debe estar cerca de la posición inicial."""
        start = [30, 20, 10, 0, 0, 0]
        frames = self._collect(start, [60, 40, 30, 0, 0, 0], velocity_pct=80)
        fk_start = frames[0]
        assert fk_start.joints_deg[0] == pytest.approx(start[0], abs=0.5)
        assert fk_start.joints_deg[1] == pytest.approx(start[1], abs=0.5)
        assert fk_start.joints_deg[2] == pytest.approx(start[2], abs=0.5)

    def test_ultimo_frame_cerca_del_destino(self):
        """El último frame debe estar cerca del destino."""
        end = [45, 30, 20, 0, 0, 0]
        frames = self._collect([0, 0, 0, 0, 0, 0], end, velocity_pct=80)
        fk_end = frames[-1]
        assert fk_end.joints_deg[0] == pytest.approx(end[0], abs=1.0)
        assert fk_end.joints_deg[1] == pytest.approx(end[1], abs=1.0)
        assert fk_end.joints_deg[2] == pytest.approx(end[2], abs=1.0)

    def test_q4_q5_q6_constantes(self):
        """Las juntas de muñeca no deben cambiar durante PTP."""
        start = [0, 0, 0, 15, 30, 45]
        frames = self._collect(start, [45, 30, 20, 15, 30, 45])
        for f in frames:
            assert f.joints_deg[3] == pytest.approx(15.0, abs=0.5)
            assert f.joints_deg[4] == pytest.approx(30.0, abs=0.5)
            assert f.joints_deg[5] == pytest.approx(45.0, abs=0.5)

    def test_cada_frame_es_fk_result(self):
        """Cada frame debe ser un ForwardKinematicsResult válido."""
        from kinematics.forward import ForwardKinematicsResult
        frames = self._collect([0, 0, 0, 0, 0, 0], [30, 20, 10, 0, 0, 0])
        for f in frames:
            assert isinstance(f, ForwardKinematicsResult)
            assert len(f.links) == 3
            assert "px" in f.position

    def test_mas_frames_con_menor_velocidad(self):
        """
        Convención heredada del MATLAB original (Vell = abs(Vel - 100)):
          velocity_pct=0   → Vell=100 → movimiento MÁS LENTO (más frames)
          velocity_pct=100 → Vell=0   → movimiento MÁS RÁPIDO (menos frames)
        """
        start, end = [0, 0, 0, 0, 0, 0], [45, 30, 20, 0, 0, 0]
        frames_rapido = self._collect(start, end, velocity_pct=100, dt=0.1)
        frames_lento  = self._collect(start, end, velocity_pct=0,   dt=0.1)
        assert len(frames_lento) > len(frames_rapido)

    def test_sin_movimiento_emite_frames(self):
        """Si inicio == fin debe emitir al menos 1 frame sin errores."""
        pos = [30, 20, 10, 0, 0, 0]
        frames = self._collect(pos, pos)
        assert len(frames) >= 1