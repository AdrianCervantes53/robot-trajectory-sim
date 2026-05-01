"""
test_api.py
Tests de integración para api/main.py.

Usa TestClient de FastAPI (síncrono) para REST y
starlette.testclient para WebSocket.

Ejecutar con: pytest tests/test_api.py -v
"""

import json
import pytest
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from fastapi.testclient import TestClient
from api.main import app
from kinematics.forward import forward_kinematics


@pytest.fixture(scope="module")
def client():
    with TestClient(app) as test_client:
        yield test_client


@pytest.fixture(autouse=True)
def reset_state(client):
    """Resetea el estado global del robot a home antes de cada test."""
    from core.state import RobotState
    from api.routers import trajectory as trajectory_router

    client.app.state.robot_state = RobotState.from_home()
    trajectory_router._saved_trajectories.clear()
    yield


# ─────────────────────────────────────────────────────────────
# GET /state
# ─────────────────────────────────────────────────────────────

class TestGetState:
    def test_retorna_200(self, client):
        r = client.get("/state")
        assert r.status_code == 200

    def test_tiene_claves_requeridas(self, client):
        r = client.get("/state")
        data = r.json()
        for key in ["joints_deg", "position", "links", "trajectory", "gripper_open"]:
            assert key in data

    def test_home_tiene_6_angulos(self, client):
        r = client.get("/state")
        assert len(r.json()["joints_deg"]) == 6


# ─────────────────────────────────────────────────────────────
# POST /joints
# ─────────────────────────────────────────────────────────────

class TestMoveJoints:
    def test_mueve_a_angulos_dados(self, client):
        r = client.post("/joints", json={"joints_deg": [30, 20, 10, 0, 0, 0]})
        assert r.status_code == 200
        data = r.json()
        assert data["joints_deg"][0] == pytest.approx(30.0, abs=0.1)
        assert data["joints_deg"][1] == pytest.approx(20.0, abs=0.1)

    def test_actualiza_posicion(self, client):
        fk = forward_kinematics(30, 20, 10, 0, 0, 0)
        r = client.post("/joints", json={"joints_deg": [30, 20, 10, 0, 0, 0]})
        data = r.json()
        assert data["position"]["px"] == pytest.approx(fk.position["px"], abs=0.1)

    def test_graba_punto_en_trayectoria(self, client):
        client.post("/joints", json={"joints_deg": [30, 20, 10, 0, 0, 0]})
        r = client.get("/state")
        assert len(r.json()["trajectory"]) >= 1

    def test_menos_de_6_angulos_falla(self, client):
        r = client.post("/joints", json={"joints_deg": [30, 20]})
        assert r.status_code == 422

    def test_mas_de_6_angulos_falla(self, client):
        r = client.post("/joints", json={"joints_deg": [0]*7})
        assert r.status_code == 422


# ─────────────────────────────────────────────────────────────
# POST /pose
# ─────────────────────────────────────────────────────────────

class TestMovePose:
    def _reachable_pose(self):
        """Genera una pose alcanzable usando FK como referencia."""
        fk = forward_kinematics(30, 20, 10, 0, 0, 0)
        return fk.position

    def test_mueve_a_pose_alcanzable(self, client):
        pos = self._reachable_pose()
        r = client.post("/pose", json=pos)
        assert r.status_code == 200

    def test_retorna_posicion_aproximada(self, client):
        pos = self._reachable_pose()
        r = client.post("/pose", json=pos)
        data = r.json()
        assert data["position"]["px"] == pytest.approx(pos["px"], abs=0.5)
        assert data["position"]["py"] == pytest.approx(pos["py"], abs=0.5)
        assert data["position"]["pz"] == pytest.approx(pos["pz"], abs=0.5)

    def test_singularidad_retorna_422(self, client):
        r = client.post("/pose", json={"px": 999, "py": 999, "pz": 999})
        assert r.status_code == 422

    def test_422_tiene_detalle(self, client):
        r = client.post("/pose", json={"px": 999, "py": 999, "pz": 999})
        assert "detail" in r.json()


# ─────────────────────────────────────────────────────────────
# POST /gripper
# ─────────────────────────────────────────────────────────────

class TestGripper:
    def test_cerrar_gripper(self, client):
        r = client.post("/gripper", json={"open": False})
        assert r.status_code == 200
        assert r.json()["gripper_open"] is False

    def test_abrir_gripper(self, client):
        client.post("/gripper", json={"open": False})
        r = client.post("/gripper", json={"open": True})
        assert r.json()["gripper_open"] is True


# ─────────────────────────────────────────────────────────────
# POST /config
# ─────────────────────────────────────────────────────────────

class TestConfig:
    def test_actualiza_velocidad(self, client):
        r = client.post("/config", json={"velocity_pct": 75})
        assert r.status_code == 200
        assert r.json()["velocity_pct"] == 75.0

    def test_actualiza_duracion(self, client):
        r = client.post("/config", json={"trajectory_duration": 3.5})
        assert r.json()["trajectory_duration"] == 3.5

    def test_actualiza_ambos(self, client):
        r = client.post("/config", json={"velocity_pct": 20, "trajectory_duration": 1.5})
        data = r.json()
        assert data["velocity_pct"] == 20.0
        assert data["trajectory_duration"] == 1.5

    def test_velocity_fuera_de_rango(self, client):
        r = client.post("/config", json={"velocity_pct": 150})
        assert r.status_code == 422


# ─────────────────────────────────────────────────────────────
# Trayectorias guardadas
# ─────────────────────────────────────────────────────────────

class TestSavedTrajectories:
    def _seed_trajectory(self, client):
        """Mueve el robot para generar historial."""
        client.post("/joints", json={"joints_deg": [30, 20, 10, 0, 0, 0]})
        client.post("/joints", json={"joints_deg": [45, 30, 15, 0, 0, 0]})

    def test_guardar_trayectoria(self, client):
        self._seed_trajectory(client)
        r = client.post("/trajectory/trajectory/save", json={"name": "test_traj"})
        assert r.status_code == 200
        assert r.json()["saved"] == "test_traj"

    def test_guardar_sin_historial_falla(self, client):
        r = client.post("/trajectory/trajectory/save", json={"name": "vacia"})
        assert r.status_code == 400

    def test_listar_trayectorias(self, client):
        self._seed_trajectory(client)
        client.post("/trajectory/trajectory/save", json={"name": "traj_a"})
        r = client.get("/trajectory/list")
        assert "traj_a" in r.json()["trajectories"]

    def test_cargar_trayectoria(self, client):
        self._seed_trajectory(client)
        client.post("/trajectory/trajectory/save", json={"name": "mi_traj"})
        client.post("/trajectory/clear")
        r = client.post("/trajectory/load", json={"name": "mi_traj"})
        assert r.status_code == 200
        assert r.json()["points"] >= 1

    def test_cargar_inexistente_falla(self, client):
        r = client.post("/trajectory/load", json={"name": "no_existe"})
        assert r.status_code == 404

    def test_clear_vacia_historial(self, client):
        self._seed_trajectory(client)
        client.post("/trajectory/clear")
        r = client.get("/state")
        assert r.json()["trajectory"] == []


# ─────────────────────────────────────────────────────────────
# WebSocket
# ─────────────────────────────────────────────────────────────

class TestWebSocket:
    def test_ptp_emite_frames(self, client):
        with client.websocket_connect("/ws") as ws:
            ws.send_text(json.dumps({
                "type": "ptp",
                "q_end": [45, 30, 20, 0, 0, 0],
            }))
            frames = []
            while True:
                data = json.loads(ws.receive_text())
                frames.append(data)
                if data["frame_type"] in ("trajectory_end", "error"):
                    break
            assert len(frames) >= 2
            assert frames[-1]["frame_type"] == "trajectory_end"

    def test_ptp_frames_tienen_estructura_correcta(self, client):
        with client.websocket_connect("/ws") as ws:
            ws.send_text(json.dumps({
                "type": "ptp",
                "q_end": [30, 20, 10, 0, 0, 0],
            }))
            while True:
                data = json.loads(ws.receive_text())
                if data["frame_type"] == "trajectory_frame":
                    assert "joints_deg" in data
                    assert "position" in data
                    assert "links" in data
                    break
                if data["frame_type"] in ("trajectory_end", "error"):
                    break

    def test_linear_emite_frames(self, client):
        fk = forward_kinematics(45, 30, 15, 0, 0, 0)
        p_end = [fk.position["px"], fk.position["py"], fk.position["pz"]]
        with client.websocket_connect("/ws") as ws:
            ws.send_text(json.dumps({"type": "linear", "p_end": p_end}))
            frames = []
            while True:
                data = json.loads(ws.receive_text())
                frames.append(data)
                if data["frame_type"] in ("trajectory_end", "error"):
                    break
            assert len(frames) >= 1

    def test_tipo_invalido_retorna_error(self, client):
        with client.websocket_connect("/ws") as ws:
            ws.send_text(json.dumps({"type": "desconocido"}))
            data = json.loads(ws.receive_text())
            assert data["frame_type"] == "error"

    def test_ptp_singularidad_retorna_error(self, client):
        """
        IK inalcanzable durante trayectoria → frame de error en algún momento.
        El primer frame puede ser válido (posición inicial alcanzable),
        pero eventualmente el generador debe emitir un frame de error.
        """
        with client.websocket_connect("/ws") as ws:
            ws.send_text(json.dumps({
                "type": "linear",
                "p_end": [999.0, 999.0, 999.0],
            }))
            frame_types = []
            # Recibir hasta encontrar error o fin (máx 20 frames)
            for _ in range(20):
                data = json.loads(ws.receive_text())
                frame_types.append(data["frame_type"])
                if data["frame_type"] in ("error", "trajectory_end"):
                    break
            assert "error" in frame_types, f"Se esperaba un frame de error, se recibieron: {frame_types}"
