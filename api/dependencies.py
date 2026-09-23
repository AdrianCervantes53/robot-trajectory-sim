from fastapi import Request
from core.state import RobotState


def get_robot_state(request: Request) -> RobotState:
    """Dependency that provides the singleton RobotState stored on app.state."""
    return request.app.state.robot_state
