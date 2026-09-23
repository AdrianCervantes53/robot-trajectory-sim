"""
trajectory.py
REST router for trajectory storage and playback operations.
"""

from fastapi import APIRouter, HTTPException, Depends

from api.schemas.trajectory import SaveTrajectoryRequest, LoadTrajectoryRequest
from api.dependencies import get_robot_state
from core.state import RobotState

router = APIRouter(prefix="/trajectory", tags=["Trajectory"])

# In-memory storage for saved trajectories
_saved_trajectories: dict[str, list] = {}


@router.post("/save", summary="Save recorded trajectory")
@router.post("/trajectory/save", summary="Save recorded trajectory (compatibility alias)")
def save_trajectory(req: SaveTrajectoryRequest, robot_state: RobotState = Depends(get_robot_state)) -> dict:
    """Saves the current end-effector trajectory history under the given name."""
    if not robot_state.trajectory:
        raise HTTPException(status_code=400, detail="No recorded trajectory found.")
    _saved_trajectories[req.name] = list(robot_state.trajectory)
    return {"saved": req.name, "points": len(robot_state.trajectory)}


@router.post("/clear", summary="Clear trajectory history")
def clear_trajectory(robot_state: RobotState = Depends(get_robot_state)) -> dict:
    """Clears the recorded trajectory points history."""
    robot_state.clear_trajectory()
    return {"trajectory": []}


@router.get("/list", summary="List saved trajectories")
def list_trajectories() -> dict:
    """Returns the names of all saved trajectories."""
    return {"trajectories": list(_saved_trajectories.keys())}


@router.post("/load", summary="Load saved trajectory")
def load_trajectory(req: LoadTrajectoryRequest, robot_state: RobotState = Depends(get_robot_state)) -> dict:
    """Loads a previously saved trajectory into the active trajectory history."""
    if req.name not in _saved_trajectories:
        raise HTTPException(
            status_code=404,
            detail=f"Trajectory '{req.name}' not found.",
        )
    robot_state.trajectory = list(_saved_trajectories[req.name])
    return {"loaded": req.name, "points": len(robot_state.trajectory)}
