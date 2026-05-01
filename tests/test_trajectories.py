"""
test_trajectories.py
Tests para trajectory/linear.py y trajectory/circular.py.

Ejecutar con: pytest tests/test_trajectories.py -v
"""

import numpy as np
import pytest
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from kinematics.forward import forward_kinematics, ForwardKinematicsResult
from trajectory.linear import linear_trajectory
from trajectory.circular import circular_trajectory, _circumcenter, _arc_direction


# ──────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────

def collect(gen):
    return list(gen)


HOME = [45, 30, 20, 0, 0, 0]

def fk_pos(q):
    return forward_kinematics(*q).position


# ──────────────────────────────────────────────
# _circumcenter
# ──────────────────────────────────────────────

class TestCircumcenter:
    def test_triangulo_conocido(self):
        """Triángulo rectángulo — circuncentro en el punto medio de la hipotenusa."""
        p1 = np.array([0.0, 0.0])
        p2 = np.array([4.0, 0.0])
        p3 = np.array([0.0, 4.0])  # ángulo recto en origen
        center, radius = _circumcenter(p1, p2, p3)
        # Circuncentro de triángulo rectángulo = punto medio hipotenusa
        np.testing.assert_allclose(center, [2.0, 2.0], atol=1e-6)
        assert radius == pytest.approx(np.sqrt(8), abs=1e-6)

    def test_circunferencia_unitaria(self):
        """3 puntos en el círculo unitario → centro en origen, radio=1."""
        p1 = np.array([1.0,  0.0])
        p2 = np.array([0.0,  1.0])
        p3 = np.array([-1.0, 0.0])
        center, radius = _circumcenter(p1, p2, p3)
        np.testing.assert_allclose(center, [0.0, 0.0], atol=1e-6)
        assert radius == pytest.approx(1.0, abs=1e-6)

    def test_puntos_colineales_lanza_error(self):
        """3 puntos colineales deben lanzar ValueError."""
        p1 = np.array([0.0, 0.0])
        p2 = np.array([1.0, 1.0])
        p3 = np.array([2.0, 2.0])
        with pytest.raises(ValueError, match="colineales"):
            _circumcenter(p1, p2, p3)

    def test_todos_los_puntos_equidistan_del_centro(self):
        """Los 3 puntos de entrada deben estar a la misma distancia del centro."""
        p1 = np.array([1.0, 3.0])
        p2 = np.array([4.0, 1.0])
        p3 = np.array([5.0, 4.0])
        center, radius = _circumcenter(p1, p2, p3)
        for p in [p1, p2, p3]:
            assert np.linalg.norm(p - center) == pytest.approx(radius, abs=1e-6)


# ──────────────────────────────────────────────
# _arc_direction
# ──────────────────────────────────────────────

class TestArcDirection:
    def test_caso1_antihorario_directo(self):
        k, t2, t3 = _arc_direction(10, 90, 170)
        assert k == +1

    def test_caso2_horario_directo(self):
        k, t2, t3 = _arc_direction(170, 90, 10)
        assert k == -1

    def test_preserva_paso_por_intermedio(self):
        """El ángulo intermedio ajustado debe estar entre theta1 y theta3_adj."""
        k, t2_adj, t3_adj = _arc_direction(10, 90, 170)
        assert min(10, t3_adj) <= t2_adj <= max(10, t3_adj)


# ──────────────────────────────────────────────
# linear_trajectory
# ──────────────────────────────────────────────

class TestLinearTrajectory:
    def _start_end_pos(self):
        """Posiciones cartesianas alcanzables para el robot."""
        fk_a = forward_kinematics(30, 20, 10, 0, 0, 0)
        fk_b = forward_kinematics(50, 10, 15, 0, 0, 0)
        pa = [fk_a.position["px"], fk_a.position["py"], fk_a.position["pz"]]
        pb = [fk_b.position["px"], fk_b.position["py"], fk_b.position["pz"]]
        return pa, pb

    def test_emite_frames(self):
        pa, pb = self._start_end_pos()
        frames = collect(linear_trajectory(pa, pb, HOME, duration=1.0, dt=0.2))
        assert len(frames) >= 2

    def test_cada_frame_es_fk_result(self):
        pa, pb = self._start_end_pos()
        frames = collect(linear_trajectory(pa, pb, HOME, duration=0.5, dt=0.1))
        for f in frames:
            assert isinstance(f, ForwardKinematicsResult)

    def test_mas_pasos_con_dt_menor(self):
        pa, pb = self._start_end_pos()
        f_grueso = collect(linear_trajectory(pa, pb, HOME, duration=1.0, dt=0.5))
        f_fino   = collect(linear_trajectory(pa, pb, HOME, duration=1.0, dt=0.1))
        assert len(f_fino) > len(f_grueso)

    def test_q4_q5_q6_constantes(self):
        """Las juntas de orientación no deben cambiar durante la trayectoria."""
        pa, pb = self._start_end_pos()
        q_init = [30, 20, 10, 15, 25, 35]
        frames = collect(linear_trajectory(pa, pb, q_init, duration=0.5, dt=0.1))
        for f in frames:
            assert f.joints_deg[3] == pytest.approx(15.0, abs=1.0)
            assert f.joints_deg[4] == pytest.approx(25.0, abs=1.0)
            assert f.joints_deg[5] == pytest.approx(35.0, abs=1.0)

    def test_links_continuos_en_cada_frame(self):
        """Cada frame debe tener eslabones conectados."""
        pa, pb = self._start_end_pos()
        frames = collect(linear_trajectory(pa, pb, HOME, duration=0.5, dt=0.2))
        for f in frames:
            assert f.links[0]["to"] == pytest.approx(f.links[1]["from"], abs=1e-5)
            assert f.links[1]["to"] == pytest.approx(f.links[2]["from"], abs=1e-5)


# ──────────────────────────────────────────────
# circular_trajectory
# ──────────────────────────────────────────────

class TestCircularTrajectory:
    def _three_arc_points(self, plane=1):
        """
        Genera 3 puntos en un arco circular alcanzable por el robot.
        Los puntos están calculados para estar dentro del espacio de trabajo.
        """
        # Radio y centro pequeños centrados cerca del origen del robot
        r, xc, yc = 3.0, 4.0, 0.0
        # 3 puntos a 0°, 90°, 180° sobre el arco
        angles = [0, 90, 180]
        points = []
        for a in angles:
            arc_x = r * np.cos(np.deg2rad(a)) + xc
            arc_y = r * np.sin(np.deg2rad(a)) + yc
            if plane == 1:   # XY, z constante
                points.append([arc_x, arc_y, 5.0])
            elif plane == 2: # XZ, y constante
                points.append([arc_x, 0.0, arc_y + 5.0])
            else:            # YZ, x constante
                points.append([4.0, arc_x, arc_y + 5.0])
        return points[0], points[1], points[2]

    def test_emite_frames_plano_xy(self):
        p0, pm, pf = self._three_arc_points(plane=1)
        frames = collect(circular_trajectory(p0, pm, pf, HOME, plane=1, step_deg=10))
        assert len(frames) >= 3

    def test_cada_frame_es_fk_result(self):
        p0, pm, pf = self._three_arc_points(plane=1)
        frames = collect(circular_trajectory(p0, pm, pf, HOME, plane=1, step_deg=15))
        for f in frames:
            assert isinstance(f, ForwardKinematicsResult)

    def test_mas_frames_con_step_menor(self):
        """Paso angular más pequeño → más frames."""
        p0, pm, pf = self._three_arc_points(plane=1)
        f_grueso = collect(circular_trajectory(p0, pm, pf, HOME, plane=1, step_deg=20))
        f_fino   = collect(circular_trajectory(p0, pm, pf, HOME, plane=1, step_deg=5))
        assert len(f_fino) > len(f_grueso)

    def test_q4_q5_q6_constantes(self):
        """Las juntas de orientación no deben cambiar durante el arco."""
        p0, pm, pf = self._three_arc_points(plane=1)
        q_init = [45, 30, 20, 10, 20, 30]
        frames = collect(circular_trajectory(p0, pm, pf, q_init, plane=1, step_deg=15))
        for f in frames:
            assert f.joints_deg[3] == pytest.approx(10.0, abs=1.5)
            assert f.joints_deg[4] == pytest.approx(20.0, abs=1.5)
            assert f.joints_deg[5] == pytest.approx(30.0, abs=1.5)

    def test_puntos_colineales_lanza_error(self):
        """3 puntos colineales deben lanzar ValueError."""
        p0 = [1.0, 0.0, 5.0]
        pm = [2.0, 0.0, 5.0]
        pf = [3.0, 0.0, 5.0]
        with pytest.raises(ValueError, match="colineales"):
            collect(circular_trajectory(p0, pm, pf, HOME, plane=1))