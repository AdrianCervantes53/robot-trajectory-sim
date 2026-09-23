from pydantic import BaseModel, Field


class JointsRequest(BaseModel):
    """Move robot to specific joint angles (forward kinematics)."""
    joints_deg: list[float] = Field(
        ...,
        min_length=6,
        max_length=6,
        description="Joint angles [q1..q6] in degrees.",
        examples=[[45, 30, 20, 0, 0, 0]],
    )


class PoseRequest(BaseModel):
    """Move end-effector to Cartesian coordinates (inverse kinematics)."""
    px: float = Field(..., description="Target X coordinate.")
    py: float = Field(..., description="Target Y coordinate.")
    pz: float = Field(..., description="Target Z coordinate.")


class GripperRequest(BaseModel):
    """Control gripper state."""
    open: bool = Field(..., description="True = open, False = closed.")


class ConfigRequest(BaseModel):
    """Update speed and duration parameters for trajectories."""
    velocity_pct: float | None = Field(
        None, ge=0, le=100,
        description="PTP speed percentage (0=slow, 100=fast).",
    )
    trajectory_duration: float | None = Field(
        None, gt=0,
        description="LIN/CIR duration in seconds.",
    )
