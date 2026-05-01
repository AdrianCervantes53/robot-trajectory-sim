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

from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from core.state import RobotState

from api.routers import movement, trajectory, ws_trajectory


@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.robot_state = RobotState.from_home()
    print("Robot simulator API iniciada. Estado: home.")
    yield
    print("API detenida.")

app = FastAPI(
    title="Robot 6-DOF Simulator API",
    description="Cinemática directa/inversa y trayectorias PTP, LIN y CIR.",
    version="1.0.0",
    lifespan=lifespan,
)

app.include_router(movement.router)
app.include_router(trajectory.router)
app.include_router(ws_trajectory.router)

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
