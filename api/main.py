"""
main.py
API REST + WebSocket del simulador de robot 6-DOF.

Endpoints REST (operaciones instantáneas)
-----------------------------------------
GET  /state                   → estado completo actual del robot
POST /joints                  → mover a ángulos de junta específicos
POST /pose                    → mover a pose cartesiana (IK)
POST /gripper                 → abrir/cerrar gripper
POST /trajectory/save         → guardar trayectoria grabada como JSON
POST /trajectory/load         → cargar y ejecutar una trayectoria guardada
POST /trajectory/clear        → limpiar historial de trayectoria
POST /config                  → actualizar velocidad y duración

WebSocket (trayectorias en tiempo real)
----------------------------------------
WS /ws

El cliente envía un mensaje JSON con el tipo de trayectoria a ejecutar.
El servidor emite un frame JSON por cada paso calculado.

Protocolo de mensaje entrante:
  { "type": "ptp",      "q_end": [q1..q6] }
  { "type": "linear",   "p_end": [px,py,pz] }
  { "type": "circular", "p_mid": [px,py,pz], "p_end": [px,py,pz], "plane": 1 }

Cada frame emitido es el JSON de RobotState.to_dict() con un campo extra:
  { ..., "frame_type": "trajectory_frame" | "trajectory_end" | "error" }

Nota de concurrencia
--------------------
`robot_state` es una instancia global única — diseñado para un solo
cliente a la vez (suficiente para portafolio y demo local). Para
multi-usuario se requeriría estado por sesión o por WebSocket.
"""

import json
import asyncio
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

from core.state import RobotState
from kinematics.forward import forward_kinematics
from kinematics.inverse import inverse_kinematics, SingularityError
from trajectory.ptp import ptp_trajectory
from trajectory.linear import linear_trajectory
from trajectory.circular import circular_trajectory

# ─────────────────────────────────────────────────────────────
# Estado global del robot
# ─────────────────────────────────────────────────────────────

robot_state = RobotState.from_home()

# ─────────────────────────────────────────────────────────────
# App
# ─────────────────────────────────────────────────────────────

@asynccontextmanager
async def lifespan(app: FastAPI):
    print("Robot simulator API iniciada. Estado: home.")
    yield
    print("API detenida.")

app = FastAPI(
    title="Robot 6-DOF Simulator API",
    description="Cinemática directa/inversa y trayectorias PTP, LIN y CIR.",
    version="1.0.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],   # en producción restringir al dominio del frontend
    allow_methods=["*"],
    allow_headers=["*"],
)

# Servir el frontend estático desde /frontend si existe
_frontend_path = Path(__file__).parent.parent / "frontend"
if _frontend_path.exists():
    app.mount("/app", StaticFiles(directory=str(_frontend_path), html=True), name="frontend")

# ─────────────────────────────────────────────────────────────
# Modelos Pydantic
# ─────────────────────────────────────────────────────────────

class JointsRequest(BaseModel):
    """Mover el robot a ángulos de junta específicos (cinemática directa)."""
    joints_deg: list[float] = Field(
        ...,
        min_length=6,
        max_length=6,
        description="Ángulos [q1..q6] en GRADOS.",
        examples=[[45, 30, 20, 0, 0, 0]],
    )

class PoseRequest(BaseModel):
    """Mover el EF a una pose cartesiana (cinemática inversa)."""
    px: float = Field(..., description="Posición X del efector final.")
    py: float = Field(..., description="Posición Y del efector final.")
    pz: float = Field(..., description="Posición Z del efector final.")

class GripperRequest(BaseModel):
    """Controlar el gripper."""
    open: bool = Field(..., description="True = abrir, False = cerrar.")

class ConfigRequest(BaseModel):
    """Actualizar parámetros de velocidad y duración de trayectorias."""
    velocity_pct: float | None = Field(
        None, ge=0, le=100,
        description="Velocidad PTP 0-100 (0=lento, 100=rápido).",
    )
    trajectory_duration: float | None = Field(
        None, gt=0,
        description="Duración LIN/CIR en segundos.",
    )

class SaveTrajectoryRequest(BaseModel):
    """Guardar la trayectoria grabada con un nombre."""
    name: str = Field(..., description="Identificador de la trayectoria.")

class LoadTrajectoryRequest(BaseModel):
    """Cargar una trayectoria guardada por nombre."""
    name: str

# Almacén en memoria de trayectorias guardadas
# (en producción → base de datos o archivos JSON en disco)
_saved_trajectories: dict[str, list] = {}

# ─────────────────────────────────────────────────────────────
# REST endpoints
# ─────────────────────────────────────────────────────────────

@app.get("/state", summary="Estado actual del robot")
def get_state() -> dict:
    """Retorna el estado completo del robot: juntas, posición, links y trayectoria."""
    return robot_state.to_dict()


@app.post("/joints", summary="Mover a ángulos de junta (FK)")
def move_joints(req: JointsRequest) -> dict:
    """
    Aplica cinemática directa con los ángulos recibidos y actualiza el estado.
    Equivale a mover los sliders manualmente en la interfaz MATLAB.
    """
    fk = forward_kinematics(*req.joints_deg)
    robot_state.apply_fk_result(fk)
    robot_state.record_trajectory_point()
    return robot_state.to_dict()


@app.post("/pose", summary="Mover a pose cartesiana (IK)")
def move_pose(req: PoseRequest) -> dict:
    """
    Aplica cinemática inversa para la posición cartesiana dada,
    mantiene la orientación actual del efector final.
    """
    try:
        ik = inverse_kinematics(
            req.px, req.py, req.pz,
            robot_state.orientation["alpha"],
            robot_state.orientation["beta"],
            robot_state.orientation["gamma"],
            robot_state.A1_np,
            robot_state.A2_np,
            robot_state.A3_np,
        )
    except SingularityError as e:
        raise HTTPException(status_code=422, detail=str(e))

    fk = forward_kinematics(*ik.joints_deg)
    robot_state.apply_fk_result(fk)
    robot_state.record_trajectory_point()
    return robot_state.to_dict()


@app.post("/gripper", summary="Abrir/cerrar gripper")
def control_gripper(req: GripperRequest) -> dict:
    """
    Cambia el estado del gripper. Si hay Arduino conectado, el módulo
    hardware/arduino.py envía la señal al servo (pin 11).
    """
    robot_state.gripper_open = req.open
    # TODO: llamar a arduino.gripper() cuando hardware/arduino.py esté portado
    return {"gripper_open": robot_state.gripper_open}


@app.post("/config", summary="Actualizar configuración de velocidad")
def update_config(req: ConfigRequest) -> dict:
    """Actualiza velocity_pct y/o trajectory_duration del estado global."""
    if req.velocity_pct is not None:
        robot_state.velocity_pct = req.velocity_pct
    if req.trajectory_duration is not None:
        robot_state.trajectory_duration = req.trajectory_duration
    return {
        "velocity_pct": robot_state.velocity_pct,
        "trajectory_duration": robot_state.trajectory_duration,
    }


@app.post("/trajectory/save", summary="Guardar trayectoria grabada")
def save_trajectory(req: SaveTrajectoryRequest) -> dict:
    """
    Guarda el historial de posiciones del EF (robot_state.trajectory)
    bajo el nombre dado. Equivale a 'Guardar trayectoria' en la interfaz MATLAB.
    """
    if not robot_state.trajectory:
        raise HTTPException(status_code=400, detail="No hay trayectoria grabada.")
    _saved_trajectories[req.name] = list(robot_state.trajectory)
    return {"saved": req.name, "points": len(robot_state.trajectory)}


@app.post("/trajectory/clear", summary="Limpiar historial de trayectoria")
def clear_trajectory() -> dict:
    """Limpia el historial de posiciones del EF. Equivale a l=2 en MATLAB."""
    robot_state.clear_trajectory()
    return {"trajectory": []}


@app.get("/trajectory/list", summary="Listar trayectorias guardadas")
def list_trajectories() -> dict:
    """Retorna los nombres de todas las trayectorias guardadas."""
    return {"trajectories": list(_saved_trajectories.keys())}


@app.post("/trajectory/load", summary="Cargar trayectoria guardada")
def load_trajectory(req: LoadTrajectoryRequest) -> dict:
    """
    Carga una trayectoria guardada al historial activo.
    Equivale a 'Cargar trayectoria' en la interfaz MATLAB.
    """
    if req.name not in _saved_trajectories:
        raise HTTPException(
            status_code=404,
            detail=f"Trayectoria '{req.name}' no encontrada.",
        )
    robot_state.trajectory = list(_saved_trajectories[req.name])
    return {"loaded": req.name, "points": len(robot_state.trajectory)}


# ─────────────────────────────────────────────────────────────
# WebSocket — trayectorias en tiempo real
# ─────────────────────────────────────────────────────────────

@app.websocket("/ws")
async def websocket_trajectory(websocket: WebSocket):
    """
    WebSocket para ejecutar trayectorias en tiempo real.

    Protocolo entrante (JSON):
      PTP:      { "type": "ptp",      "q_end": [q1..q6] }
      Lineal:   { "type": "linear",   "p_end": [px,py,pz] }
      Circular: { "type": "circular", "p_mid": [px,py,pz],
                                      "p_end": [px,py,pz],
                                      "plane": 1 }

    Cada frame emitido al cliente:
      { ...RobotState.to_dict(), "frame_type": "trajectory_frame" }

    Último frame:
      { ...RobotState.to_dict(), "frame_type": "trajectory_end" }

    En caso de error:
      { "frame_type": "error", "detail": "mensaje" }
    """
    await websocket.accept()
    robot_state.clear_trajectory()

    try:
        while True:
            raw = await websocket.receive_text()
            msg = json.loads(raw)
            traj_type = msg.get("type")

            try:
                if traj_type == "ptp":
                    gen = ptp_trajectory(
                        q_start_deg=robot_state.joints_deg,
                        q_end_deg=msg["q_end"],
                        velocity_pct=robot_state.velocity_pct,
                    )

                elif traj_type == "linear":
                    gen = linear_trajectory(
                        p_start=robot_state.position_list,
                        p_end=msg["p_end"],
                        q_current_deg=robot_state.joints_deg,
                        duration=robot_state.trajectory_duration,
                    )

                elif traj_type == "circular":
                    gen = circular_trajectory(
                        p_start=robot_state.position_list,
                        p_mid=msg["p_mid"],
                        p_end=msg["p_end"],
                        q_current_deg=robot_state.joints_deg,
                        plane=msg.get("plane", 1),
                    )

                else:
                    await websocket.send_text(json.dumps({
                        "frame_type": "error",
                        "detail": f"Tipo de trayectoria desconocido: '{traj_type}'.",
                    }))
                    continue

                # Emitir frames conforme se calculan
                last_frame = None
                for fk_result in gen:
                    robot_state.apply_fk_result(fk_result)
                    robot_state.record_trajectory_point()

                    frame = robot_state.to_dict()
                    frame["frame_type"] = "trajectory_frame"
                    await websocket.send_text(json.dumps(frame, default=float))
                    last_frame = frame

                    # Ceder el event loop para no bloquear otras corrutinas
                    await asyncio.sleep(0)

                # Marcar fin de trayectoria
                if last_frame is not None:
                    last_frame["frame_type"] = "trajectory_end"
                    await websocket.send_text(json.dumps(last_frame, default=float))

            except SingularityError as e:
                await websocket.send_text(json.dumps({
                    "frame_type": "error",
                    "detail": str(e),
                }))
            except (KeyError, ValueError) as e:
                await websocket.send_text(json.dumps({
                    "frame_type": "error",
                    "detail": f"Mensaje inválido: {e}",
                }))

    except WebSocketDisconnect:
        print("Cliente WebSocket desconectado.")