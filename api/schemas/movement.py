from pydantic import BaseModel, Field

class JointsRequest(BaseModel):
    """Mover el robot a ángulos de junta específicos (cinemática directa)."""
    joints_deg: list[float] = Field(
        ...,
        min_length=6,
        max_length=6,
        description="Ángulos [q1..q6] en GRADOS.",
        examples=[[45, 30, 20, 0, 0, 0]],
    )

class PoseRequest(BaseModel):
    """Mover el EF a una pose cartesiana (cinemática inversa)."""
    px: float = Field(..., description="Posición X del efector final.")
    py: float = Field(..., description="Posición Y del efector final.")
    pz: float = Field(..., description="Posición Z del efector final.")

class GripperRequest(BaseModel):
    """Controlar el gripper."""
    open: bool = Field(..., description="True = abrir, False = cerrar.")

class ConfigRequest(BaseModel):
    """Actualizar parámetros de velocidad y duración de trayectorias."""
    velocity_pct: float | None = Field(
        None, ge=0, le=100,
        description="Velocidad PTP 0-100 (0=lento, 100=rápido).",
    )
    trajectory_duration: float | None = Field(
        None, gt=0,
        description="Duración LIN/CIR en segundos.",
    )