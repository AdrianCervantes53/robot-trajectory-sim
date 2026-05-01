"""
routers/trajectory.py

Endpoints REST (operaciones de trayecoria)
-----------------------------------------
POST /trajectory/save         → guardar trayectoria grabada como JSON
POST /trajectory/load         → cargar y ejecutar una trayectoria guardada
POST /trajectory/clear        → limpiar historial de trayectoria

"""
from fastapi import APIRouter, HTTPException, Depends

from api.schemas.trajectory import SaveTrajectoryRequest, LoadTrajectoryRequest
from api.dependencies import get_robot_state
from core.state import RobotState

router = APIRouter(prefix="/trajectory", tags=["Trajectory"])

# Almacén en memoria de trayectorias guardadas
# (en producción → base de datos o archivos JSON en disco)
_saved_trajectories: dict[str, list] = {}

@router.post("/trajectory/save", summary="Guardar trayectoria grabada")
def save_trajectory(req: SaveTrajectoryRequest, robot_state: RobotState = Depends(get_robot_state)) -> dict:
    """
    Guarda el historial de posiciones del EF (robot_state.trajectory)
    bajo el nombre dado. Equivale a 'Guardar trayectoria' en la interfaz MATLAB.
    """
    if not robot_state.trajectory:
        raise HTTPException(status_code=400, detail="No hay trayectoria grabada.")
    _saved_trajectories[req.name] = list(robot_state.trajectory)
    return {"saved": req.name, "points": len(robot_state.trajectory)}


@router.post("/clear", summary="Limpiar historial de trayectoria")
def clear_trajectory(robot_state: RobotState = Depends(get_robot_state)) -> dict:
    """Limpia el historial de posiciones del EF. Equivale a l=2 en MATLAB."""
    robot_state.clear_trajectory()
    return {"trajectory": []}


@router.get("/list", summary="Listar trayectorias guardadas")
def list_trajectories() -> dict:
    """Retorna los nombres de todas las trayectorias guardadas."""
    return {"trajectories": list(_saved_trajectories.keys())}


@router.post("/load", summary="Cargar trayectoria guardada")
def load_trajectory(req: LoadTrajectoryRequest, robot_state: RobotState = Depends(get_robot_state)) -> dict:
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