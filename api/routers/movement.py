from fastapi import APIRouter, HTTPException, Depends

from api.schemas.movement import JointsRequest, PoseRequest, GripperRequest, ConfigRequest
from api.dependencies import get_robot_state
from core.state import RobotState
from kinematics.forward import forward_kinematics
from kinematics.inverse import inverse_kinematics, SingularityError


router = APIRouter(tags=["Movement"])

@router.get("/state", summary="Estado actual del robot")
def get_state(robot_state: RobotState = Depends(get_robot_state)) -> dict:
    """Retorna el estado completo del robot: juntas, posición, links y trayectoria."""
    return robot_state.to_dict()


@router.post("/joints", summary="Mover a ángulos de junta (FK)")
def move_joints(req: JointsRequest, robot_state: RobotState = Depends(get_robot_state)) -> dict:
    """
    Aplica cinemática directa con los ángulos recibidos y actualiza el estado.
    Equivale a mover los sliders manualmente en la interfaz MATLAB.
    """
    fk = forward_kinematics(*req.joints_deg)
    robot_state.apply_fk_result(fk)
    robot_state.record_trajectory_point()
    return robot_state.to_dict()


@router.post("/pose", summary="Mover a pose cartesiana (IK)")
def move_pose(req: PoseRequest, robot_state: RobotState = Depends(get_robot_state)) -> dict:
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


@router.post("/gripper", summary="Abrir/cerrar gripper")
def control_gripper(req: GripperRequest, robot_state: RobotState = Depends(get_robot_state)) -> dict:
    """
    Cambia el estado del gripper. Si hay Arduino conectado, el módulo
    hardware/arduino.py envía la señal al servo (pin 11).
    """
    robot_state.gripper_open = req.open
    # TODO: llamar a arduino.gripper() cuando hardware/arduino.py esté portado
    return {"gripper_open": robot_state.gripper_open}


@router.post("/config", summary="Actualizar configuración de velocidad")
def update_config(req: ConfigRequest, robot_state: RobotState = Depends(get_robot_state)) -> dict:
    """Actualiza velocity_pct y/o trajectory_duration del estado global."""
    if req.velocity_pct is not None:
        robot_state.velocity_pct = req.velocity_pct
    if req.trajectory_duration is not None:
        robot_state.trajectory_duration = req.trajectory_duration
    return {
        "velocity_pct": robot_state.velocity_pct,
        "trajectory_duration": robot_state.trajectory_duration,
    }