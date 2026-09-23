"""
movement.py
REST router for robot kinematics and manual movements.
"""

from fastapi import APIRouter, HTTPException, Depends

from api.schemas.movement import JointsRequest, PoseRequest, GripperRequest, ConfigRequest
from api.dependencies import get_robot_state
from core.state import RobotState
from kinematics.forward import forward_kinematics
from kinematics.inverse import inverse_kinematics, SingularityError

router = APIRouter(tags=["Movement"])


@router.get("/state", summary="Current robot state")
def get_state(robot_state: RobotState = Depends(get_robot_state)) -> dict:
    """Returns the full robot state: joints, position, links, and trajectory."""
    return robot_state.to_dict()


@router.post("/joints", summary="Move to joint angles (FK)")
def move_joints(req: JointsRequest, robot_state: RobotState = Depends(get_robot_state)) -> dict:
    """Applies forward kinematics with the requested angles and updates state."""
    fk = forward_kinematics(*req.joints_deg)
    robot_state.apply_fk_result(fk)
    robot_state.record_trajectory_point()
    return robot_state.to_dict()


@router.post("/pose", summary="Move to Cartesian pose (IK)")
def move_pose(req: PoseRequest, robot_state: RobotState = Depends(get_robot_state)) -> dict:
    """Applies inverse kinematics for target Cartesian coordinates, keeping current orientation."""
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


@router.post("/gripper", summary="Open/close gripper")
def control_gripper(req: GripperRequest, robot_state: RobotState = Depends(get_robot_state)) -> dict:
    """Updates gripper open state and returns the updated full robot state."""
    robot_state.gripper_open = req.open
    return robot_state.to_dict()


@router.post("/config", summary="Update speed and duration configuration")
def update_config(req: ConfigRequest, robot_state: RobotState = Depends(get_robot_state)) -> dict:
    """Updates velocity_pct and/or trajectory_duration in global robot state."""
    if req.velocity_pct is not None:
        robot_state.velocity_pct = req.velocity_pct
    if req.trajectory_duration is not None:
        robot_state.trajectory_duration = req.trajectory_duration
    return {
        "velocity_pct": robot_state.velocity_pct,
        "trajectory_duration": robot_state.trajectory_duration,
    }
