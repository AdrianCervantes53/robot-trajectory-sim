"""
ws_trajectory.py
WebSocket router for streaming real-time robot trajectories.
"""

import json
import asyncio

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from kinematics.inverse import SingularityError
from trajectory.ptp import ptp_trajectory
from trajectory.linear import linear_trajectory
from trajectory.circular import circular_trajectory

router = APIRouter(tags=["WebSocket-Trajectory"])

# Time step between frames in seconds (20 FPS).
# asyncio.sleep(FRAME_DT) paces the simulation in real time according to duration and speed settings.
FRAME_DT: float = 0.05


@router.websocket("/ws")
async def websocket_trajectory(websocket: WebSocket):
    """
    WebSocket endpoint to execute and stream trajectories in real time.

    Inbound protocol (JSON):
      PTP:      { "type": "ptp",      "q_end": [q1..q6] }
      Linear:   { "type": "linear",   "p_end": [px,py,pz] }
      Circular: { "type": "circular", "p_mid": [px,py,pz],
                                      "p_end": [px,py,pz],
                                      "plane": 1 }

    Outbound stream:
      { ...RobotState.to_dict(), "frame_type": "trajectory_frame" }

    Last frame:
      { ...RobotState.to_dict(), "frame_type": "trajectory_end" }

    On error:
      { "frame_type": "error", "detail": "message" }
    """
    await websocket.accept()
    robot_state = websocket.app.state.robot_state
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
                        dt=FRAME_DT,
                    )

                elif traj_type == "linear":
                    gen = linear_trajectory(
                        p_start=robot_state.position_list,
                        p_end=msg["p_end"],
                        q_current_deg=robot_state.joints_deg,
                        duration=robot_state.trajectory_duration,
                        dt=FRAME_DT,
                    )

                elif traj_type == "circular":
                    gen = circular_trajectory(
                        p_start=robot_state.position_list,
                        p_mid=msg["p_mid"],
                        p_end=msg["p_end"],
                        q_current_deg=robot_state.joints_deg,
                        plane=msg.get("plane", 1),
                        duration=robot_state.trajectory_duration,
                        dt=FRAME_DT,
                    )

                else:
                    await websocket.send_text(json.dumps({
                        "frame_type": "error",
                        "detail": f"Unknown trajectory type: '{traj_type}'.",
                    }))
                    continue

                # Stream frames in real time with FRAME_DT pacing
                last_frame = None
                for fk_result in gen:
                    robot_state.apply_fk_result(fk_result)
                    robot_state.record_trajectory_point()

                    frame = robot_state.to_dict()
                    frame["frame_type"] = "trajectory_frame"
                    await websocket.send_text(json.dumps(frame, default=float))
                    last_frame = frame

                    await asyncio.sleep(FRAME_DT)

                # Mark end of trajectory
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
                    "detail": f"Invalid message: {e}",
                }))

    except WebSocketDisconnect:
        pass
