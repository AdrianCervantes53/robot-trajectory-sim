from pydantic import BaseModel, Field

class SaveTrajectoryRequest(BaseModel):
    """Guardar la trayectoria grabada con un nombre."""
    name: str = Field(..., description="Identificador de la trayectoria.")

class LoadTrajectoryRequest(BaseModel):
    """Cargar una trayectoria guardada por nombre."""
    name: str