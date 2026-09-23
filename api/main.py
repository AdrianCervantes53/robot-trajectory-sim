"""
main.py
FastAPI entry point for 6-DOF Robot Trajectory Simulator.
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
    print("Robot simulator API started. State: home.")
    yield
    print("Robot simulator API stopped.")


app = FastAPI(
    title="Robot 6-DOF Simulator API",
    description="Forward/inverse kinematics and PTP, LIN, CIR trajectory simulation.",
    version="1.0.0",
    lifespan=lifespan,
)

app.include_router(movement.router)
app.include_router(trajectory.router)
app.include_router(ws_trajectory.router)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

_frontend_path = Path(__file__).parent.parent / "frontend"
if _frontend_path.exists():
    app.mount("/app", StaticFiles(directory=str(_frontend_path), html=True), name="frontend")
