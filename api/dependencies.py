from fastapi import Request
from core.state import RobotState

def get_robot_state(request: Request) -> RobotState:
    return request.app.state.robot_state
